// SPDX-License-Identifier: GPL-2.0
/*
 * xm530_mve: encoder sessions, and the firmware's message queues (host
 * protocol 2).
 *
 * Locks, in the order they may nest: mve_rsrc.ko's scheduler semaphore, which
 * it holds when it runs the callbacks below; sessions_lock; a session's lock.
 * So mver_scheduler_execute(), _stop() and _(un)register_session() are called
 * with no lock of ours held. mve_base.ko uses interruptible semaphores: when a
 * signal ends the wait it goes on without one, and releases it all the same.
 */

#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "mve.h"

#define ROLE "video_encoder.hevc"
#define FMT_NV12 0x7f000101
#define FMT_HEVC 0x7f000107

/* Requests to the firmware, and its answers */
#define REQ_GO 1001
#define REQ_STOP 1002
#define REQ_INPUT_FLUSH 1003
#define REQ_OUTPUT_FLUSH 1004
#define REQ_SWITCH 1005
#define REQ_PING 1006
#define REQ_JOB 1009
#define REQ_SET_OPTION 1010
#define REQ_RELEASE_REF 1011
#define RSP_SWITCHED_IN 2001
#define RSP_SWITCHED_OUT 2002
#define RSP_OPTION_SET 2003
#define RSP_JOB_DEQUEUED 2004
#define RSP_INPUT 2005
#define RSP_OUTPUT 2006
#define RSP_INPUT_FLUSHED 2007
#define RSP_OUTPUT_FLUSHED 2008
#define RSP_PONG 2009
#define RSP_ERROR 2010
#define RSP_STATE_CHANGE 2011
#define RSP_DUMP 2012
#define RSP_IDLE 2013
#define RSP_FRAME_ALLOC 2014
#define RSP_SEQUENCE 2015
#define RSP_EVENT 2016
#define RSP_OPTION_FAILED 2017
#define RSP_REF_UNUSED 2018
#define BUF_FRAME 3001
#define BUF_BITSTREAM 3002
#define BUF_PARAM 3003

/* Events to user space */
#define EV_SWITCHED_IN 1
#define EV_SWITCHED_OUT 2
#define EV_PONG 3
#define EV_STATE_CHANGED 4
#define EV_ERROR 5
#define EV_STREAM_ERROR 6
#define EV_PROCESSED 7
#define EV_INPUT 8
#define EV_OUTPUT 9
#define EV_SET 11
#define EV_INPUT_FLUSHED 12
#define EV_OUTPUT_FLUSHED 13
#define EV_DUMP 14
#define EV_HUNG 15
#define EV_FRAME_ALLOC 16
#define EV_SEQUENCE 17
#define EV_BUF_PARAM 18
#define EV_REF_UNUSED 19

/* A page each way, 1020 words of messages after a header */
struct q_host {
	u16 out_rpos, in_wpos;
	u32 reserved[3];
	u32 data[1020];
};
struct q_mve {
	u16 out_wpos, in_rpos;
	u32 reserved[3];
	u32 data[1020];
};
#define Q_WORDS 1020

struct queue {
	struct mve_dma *host, *mve;
};

struct rpc {
	u32 state; /* 0 free, 1 a call, 2 its return */
	u32 id; /* 1 print, 2 allocate, 3 resize, 4 free */
	u32 size;
	u32 param[1021];
};
#define RPC_CALL 1
#define RPC_RETURN 2

/* The buffers' messages */
struct msg_frame {
	u64 handle, tag;
	u32 flags;
	u16 visible_height, visible_width;
	u16 format, pad;
	u32 top[3], bottom[3];
	s32 stride[3];
	u16 width, height;
	u32 pad2;
};
struct msg_bitstream {
	u64 handle, tag;
	u32 flags, size, offset, filled, addr, pad;
};

struct buffer {
	struct list_head node;
	struct list_head flush; /* while a quick flush holds it */
	struct mve_user_buffer user; /* as registered, then as filled */
	u32 va; /* of the byte at user.addr */
	u32 map; /* of its first page */
	u32 port; /* 0 input, 1 output */
	int id; /* the firmware's name for it, -1 for none */
	int in_use;
};

struct pending {
	struct list_head node;
	struct mve_user_queue q;
};

/* Where a session is in the hardware */
enum { OUT, GOING_IN, GOING_OUT, WAITING, RUNNING, STATES };
static const char *const state_name[STATES] = {"out", "going in", "going out",
					       "waiting", "running"};
static const u8 state_ok[STATES][STATES] = {
	[OUT] = {1, 1, 0, 0, 1},
	[GOING_IN] = {0, 0, 0, 1, 1},
	[GOING_OUT] = {1, 0, 0, 0, 0},
	[WAITING] = {1, 0, 0, 0, 1},
	[RUNNING] = {1, 0, 1, 1, 1},
};

/* Where it is in idleness: told by the firmware, acknowledged, idle */
enum { NOT_IDLE, IDLE_1, IDLE_2, IDLE };

/* What the firmware is doing; NONE: no change asked for */
#define FW_STOPPED 0
#define FW_RUNNING 2
#define FW_NONE 4

enum { WD_NONE, WD_PINGED, WD_FINE, WD_HUNG };
#define WD_PERIOD 3000 /* ms */

struct mve_session {
	struct list_head node; /* sessions */
	struct kref kref;
	struct file *file;
	struct mutex lock;
	struct mve_mmu *mmu;
	struct mve_fw *fw;
	bool activated;
	bool registered; /* with the scheduler; under lock */
	struct mutex config_lock; /* one option and its answer at a time */
	int state;
	u32 fw_state, fw_want;
	int idle;
	bool keep; /* something came in since it went idle */
	bool out_wanted; /* to be switched out when it answers */
	bool one_frame; /* another session waits */
	bool eos;
	bool no_output_yet;
	int wd_state, wd_pings;
	unsigned long wd_time;
	u32 flush, flushed; /* a quick flush: asked, done */
	u32 n_input, n_output; /* buffers the firmware holds */
	int answers; /* it owes us */
	u16 next_id;
	struct queue msg, in, out;
	struct mve_dma *rpc;
	struct list_head pending_in, pending_out;
	struct list_head buffers;
	struct list_head flush_list;
	struct mve_event_queue events, responses;
};

static LIST_HEAD(sessions);
static DEFINE_MUTEX(sessions_lock);
static int num_sessions;
static struct device *mve_dev;
static void watchdog(struct work_struct *work);
static DECLARE_DELAYED_WORK(watchdog_work, watchdog);

static void free_session(struct kref *kref);

/* A reference, if the session has not been destroyed */
static bool get_session(struct mve_session *s)
{
	struct mve_session *i;
	bool found = false;

	mutex_lock(&sessions_lock);
	list_for_each_entry(i, &sessions, node)
		if (i == s) {
			kref_get(&s->kref);
			found = true;
			break;
		}
	mutex_unlock(&sessions_lock);
	return found;
}

void mve_session_put(struct mve_session *s)
{
	kref_put(&s->kref, free_session);
}

struct mve_session *mve_session_get(struct file *file)
{
	struct mve_session *s, *found = NULL;

	mutex_lock(&sessions_lock);
	list_for_each_entry(s, &sessions, node)
		if (s->file == file) {
			kref_get(&s->kref);
			found = s;
			break;
		}
	mutex_unlock(&sessions_lock);
	return found;
}

static bool set_state(struct mve_session *s, int state)
{
	if (!state_ok[s->state][state]) {
		pr_debug("mve: session %s cannot become %s\n",
			 state_name[s->state], state_name[state]);
		return false;
	}
	s->state = state;
	return true;
}

static void answered(struct mve_session *s)
{
	WARN_ON_ONCE(!set_state(s, RUNNING));
}

/*
 * Messages. The host writes into its own page at in_wpos, up to the
 * firmware's in_rpos, and reads from the firmware's page at out_rpos. The
 * pages are uncached.
 */
static int q_write(struct mve_session *s, struct queue *q, u16 code, u16 size,
		   const void *data)
{
	struct q_host *h = mve_rsrc_dma_mem_map(q->host);
	struct q_mve *m = mve_rsrc_dma_mem_map(q->mve);
	u32 w = ACCESS_ONCE(h->in_wpos), r = ACCESS_ONCE(m->in_rpos);
	u32 words = 1 + DIV_ROUND_UP(size, 4), i, v;

	if (w >= Q_WORDS || r >= Q_WORDS)
		return MVE_HW;
	/* a full queue would read as an empty one */
	if (words >= (r > w ? r - w : r + Q_WORDS - w))
		return MVE_TIMEOUT;
	h->data[w] = code | (u32)size << 16;
	w = (w + 1) % Q_WORDS;
	for (i = 0; i < words - 1; i++) {
		v = 0;
		memcpy(&v, (const u8 *)data + 4 * i,
		       min_t(u32, 4, size - 4 * i));
		h->data[w] = v;
		w = (w + 1) % Q_WORDS;
	}
	wmb();
	h->in_wpos = w;
	wmb();
	if ((code >= REQ_GO && code <= REQ_OUTPUT_FLUSH) ||
	    code == REQ_SET_OPTION)
		s->answers++;
	return MVE_OK;
}

/* The next message, its size at most max; false when there is none */
static bool q_read(struct mve_session *s, struct queue *q, u16 *code,
		   u16 *size, void *data, u32 max)
{
	struct q_host *h = mve_rsrc_dma_mem_map(q->host);
	struct q_mve *m = mve_rsrc_dma_mem_map(q->mve);
	u32 r = ACCESS_ONCE(h->out_rpos), w = ACCESS_ONCE(m->out_wpos);
	u32 n, head, i, words, *p = data;

	if (r >= Q_WORDS || w >= Q_WORDS || r == w)
		return false;
	rmb();
	n = w >= r ? w - r : w + Q_WORDS - r;
	head = m->data[r];
	*code = head;
	*size = head >> 16;
	words = DIV_ROUND_UP(*size, 4);
	if (words > n - 1 || *size > max)
		return false;
	for (i = 0; i < words; i++)
		p[i] = m->data[(r + 1 + i) % Q_WORDS];
	wmb();
	h->out_rpos = (r + 1 + words) % Q_WORDS;
	wmb();
	switch (*code) {
	case RSP_OPTION_SET:
	case RSP_INPUT_FLUSHED:
	case RSP_OUTPUT_FLUSHED:
	case RSP_STATE_CHANGE:
	case RSP_OPTION_FAILED:
		s->answers--;
		break;
	}
	return true;
}

static int request(struct mve_session *s, u16 code, u16 size, const void *data)
{
	int ret;

	if (!s->fw)
		return MVE_NOT_ACTIVE;
	ret = q_write(s, &s->msg, code, size, data);
	if (code != REQ_SWITCH)
		s->idle = NOT_IDLE;
	return ret;
}

/* Puts the session in the hardware if it is out: no lock held */
static bool schedule_session(struct mve_session *s)
{
	bool ok = true;

	if (!s->registered)
		return false;
	if (s->state == OUT)
		ok = mver_scheduler_execute(s);
	return s->state != OUT && s->state != GOING_OUT && ok;
}

/* Asks the firmware to leave; under the session's lock, perhaps within a
 * callback of the scheduler */
static void switch_out(struct mve_session *s)
{
	if (s->state != RUNNING) {
		if (s->state == GOING_IN || s->state == WAITING)
			s->out_wanted = true;
		return;
	}
	set_state(s, GOING_OUT);
	request(s, REQ_SWITCH, 0, NULL);
	mver_irq_signal_mve(s);
}

/*
 * Buffers
 */
static struct buffer *find_buffer(struct mve_session *s, u64 handle)
{
	struct buffer *b;

	list_for_each_entry(b, &s->buffers, node)
		if (b->user.handle == handle)
			return b;
	return NULL;
}

/* The pages at a user address: MMZ, mapped by /dev/mmz (VM_PFNMAP), so no
 * reference is taken and the caches are user space's business, as they were
 * with mve_base.ko */
static u32 *user_pages(unsigned long addr, u32 n)
{
	struct vm_area_struct *vma;
	unsigned long pfn;
	u32 *pages, i;

	pages = kmalloc(4 * n, GFP_KERNEL);
	if (!pages)
		return NULL;
	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, addr);
	if (!vma || addr < vma->vm_start ||
	    n > (vma->vm_end - addr) >> PAGE_SHIFT)
		goto fail;
	for (i = 0; i < n; i++) {
		if (follow_pfn(vma, addr + ((unsigned long)i << PAGE_SHIFT),
			       &pfn))
			goto fail;
		pages[i] = pfn << PAGE_SHIFT;
	}
	up_read(&current->mm->mmap_sem);
	return pages;
fail:
	up_read(&current->mm->mmap_sem);
	kfree(pages);
	return NULL;
}

static bool buffer_valid(const struct mve_user_buffer *b)
{
	u64 pixels;

	if (!b->addr || b->addr != (unsigned long)b->addr) {
		/* 32-bit user space may pass the address sign-extended */
		if (!b->addr || (s64)b->addr != (s32)b->addr)
			return false;
	}
	if (!b->size || b->size > 64 << 20)
		return false;
	if (b->format == FMT_HEVC)
		return true;
	if (b->format != FMT_NV12)
		return false;
	if (!b->width || !b->height || b->stride < b->width ||
	    b->width > 8192 || b->height > 8192 || b->stride > 16384)
		return false;
	pixels = (u64)b->width * b->height;
	return b->size >= 3 * pixels / 2 &&
	       b->size >= (u64)b->stride * b->height;
}

int mve_session_register(struct mve_session *s, u32 port,
			 const struct mve_user_buffer *u)
{
	u32 off, n, *pages;
	struct buffer *b;
	int ret = MVE_NOMEM;

	/* MMZ only; no CRC buffer */
	if (u->allocator || u->crc_addr)
		return MVE_NOT_IMPLEMENTED;
	if (!buffer_valid(u))
		return MVE_BAD_PARAM;
	mutex_lock(&s->lock);
	if (find_buffer(s, u->handle)) {
		mutex_unlock(&s->lock);
		return MVE_OK;
	}
	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		goto out;
	off = (u32)u->addr & ~PAGE_MASK;
	n = PAGE_ALIGN(off + u->size) >> PAGE_SHIFT;
	pages = user_pages((u32)u->addr & PAGE_MASK, n);
	if (!pages)
		goto out_free;
	/* an encoder's input are frames, its output a bitstream */
	b->map = mve_mmu_map_in(s->mmu,
				&mve_regions[port ? MVE_REGION_BITSTREAM :
						    MVE_REGION_FRAMES],
				pages, n, n, port ? 0 : 0x1000000, false);
	kfree(pages);
	if (!b->map)
		goto out_free;
	b->user = *u;
	b->va = b->map + off;
	b->port = port;
	b->id = -1;
	INIT_LIST_HEAD(&b->flush);
	list_add(&b->node, &s->buffers);
	mutex_unlock(&s->lock);
	return MVE_OK;
out_free:
	kfree(b);
out:
	mutex_unlock(&s->lock);
	return ret;
}

static void drop_buffer(struct mve_session *s, struct buffer *b)
{
	list_del(&b->flush);
	list_del(&b->node);
	mve_mmu_unmap(s->mmu, b->map);
	kfree(b);
}

int mve_session_unregister(struct mve_session *s, u64 handle)
{
	struct buffer *b;
	int ret = MVE_OK;

	mutex_lock(&s->lock);
	b = find_buffer(s, handle);
	if (!b) {
		ret = MVE_BAD_PARAM;
	} else if (b->in_use) {
		ret = MVE_BUSY;
	} else {
		drop_buffer(s, b);
		if (s->registered)
			mver_scheduler_flush_tlb(s);
	}
	mutex_unlock(&s->lock);
	return ret;
}

/* Regions of interest go before the picture they are for */
static int write_roi(struct mve_session *s, const struct mve_user_queue *q)
{
	u8 msg[200] = {0};
	int i;

	if (q->nroi > ARRAY_SIZE(q->roi))
		return MVE_BAD_PARAM;
	msg[0] = 3;
	msg[4] = q->nroi;
	for (i = 0; i < q->nroi; i++)
		memcpy(msg + 8 + 12 * i, q->roi[i], sizeof(q->roi[i]));
	return q_write(s, &s->in, BUF_PARAM, sizeof(msg), msg);
}

static int write_frame(struct mve_session *s, struct buffer *b,
		       const struct mve_user_queue *q)
{
	const struct mve_user_buffer *u = &b->user;
	u32 align = u->stride_align ? u->stride_align : 1;
	u32 mf = q->filled ? q->mve_flags | 4 : q->mve_flags & ~0xcu;
	struct msg_frame f;
	s16 stride;
	u32 aligned;

	if (q->mve_flags & 1) /* interlaced */
		return MVE_NOT_IMPLEMENTED;
	/* the firmware's stride is 16 bits, and both planes must lie in what
	 * was mapped */
	if (align & (align - 1) || align > 4096)
		return MVE_BAD_PARAM;
	aligned = ALIGN(u->stride, align);
	if (aligned > S16_MAX || (u64)aligned * u->height +
					 (u64)aligned * DIV_ROUND_UP(u->height, 2) >
					 u->size)
		return MVE_BAD_PARAM;
	stride = aligned;
	memset(&f, 0, sizeof(f));
	f.handle = (u16)b->id;
	f.tag = q->timestamp;
	f.flags = (mf & 0x3f) | (q->flags & 1 ? 0x10000 : 0) |
		  (q->flags & 4 ? 0x4000 : 0) | (q->flags & 8 ? 0x2000 : 0) |
		  (q->flags & 0x200 ? 0x8000 : 0) | (mf & 0x100 ? 0x40 : 0) |
		  (mf & 0x200 ? 0x80 : 0);
	f.visible_height = f.height = u->height;
	f.visible_width = f.width = u->width;
	f.format = 0x201; /* NV12 */
	f.top[0] = b->va;
	f.top[1] = b->va + u->height * stride;
	f.stride[0] = f.stride[1] = stride;
	return q_write(s, b->port ? &s->out : &s->in, BUF_FRAME, sizeof(f),
		       &f);
}

static int write_bitstream(struct mve_session *s, struct buffer *b,
			   const struct mve_user_queue *q)
{
	struct msg_bitstream m;

	if (q->filled > b->user.size)
		return MVE_BAD_PARAM;
	memset(&m, 0, sizeof(m));
	m.handle = (u16)b->id;
	m.tag = q->timestamp;
	m.flags = q->flags & 0x4b1;
	m.size = b->user.size;
	m.filled = q->filled;
	m.addr = b->va;
	return q_write(s, b->port ? &s->out : &s->in, BUF_BITSTREAM, sizeof(m),
		       &m);
}

static int give_buffer(struct mve_session *s, const struct mve_user_queue *q,
		       bool input)
{
	struct list_head *pending = input ? &s->pending_in : &s->pending_out;
	struct buffer *b;
	int ret;

	if (!list_empty(pending))
		return MVE_TIMEOUT; /* after those */
	b = find_buffer(s, q->handle);
	if (!b || b->port != !input)
		return MVE_BAD_PARAM;
	if (!s->fw)
		return MVE_NOT_ACTIVE;
	b->user.filled = q->filled;
	b->user.flags = q->flags;
	b->user.mve_flags = q->mve_flags;
	b->user.timestamp = q->timestamp;
	if (b->id < 0)
		b->id = s->next_id++;
	if ((q->mve_flags & 0x20000) && q->nroi && input) {
		ret = write_roi(s, q);
		if (ret)
			goto fail;
	}
	ret = b->user.format == FMT_HEVC ? write_bitstream(s, b, q) :
					   write_frame(s, b, q);
	if (!ret) {
		if (input)
			s->n_input++;
		else
			s->n_output++;
		b->in_use++;
		return MVE_OK;
	}
fail:
	if (!b->in_use)
		b->id = -1;
	return ret;
}

static void return_pending(struct mve_session *s, struct list_head *list,
			   bool input)
{
	struct pending *p, *tmp;
	struct mve_user_buffer u;

	list_for_each_entry_safe(p, tmp, list, node) {
		memset(&u, 0, sizeof(u));
		u.addr = p->q.addr;
		u.flags = p->q.flags;
		u.mve_flags = p->q.mve_flags;
		mve_queue_add(&s->events, input ? EV_INPUT : EV_OUTPUT,
			      sizeof(u), &u);
		list_del(&p->node);
		kfree(p);
	}
}

/* The buffers that did not fit the firmware's queue before */
static void give_pending(struct mve_session *s, bool input)
{
	struct list_head *list = input ? &s->pending_in : &s->pending_out;
	struct mve_user_buffer u;
	struct pending *p;
	LIST_HEAD(todo);
	int ret;

	list_splice_init(list, &todo);
	while (!list_empty(&todo)) {
		p = list_first_entry(&todo, struct pending, node);
		ret = give_buffer(s, &p->q, input);
		if (ret == MVE_TIMEOUT)
			break;
		if (ret) {
			memset(&u, 0, sizeof(u));
			u.addr = p->q.addr;
			u.flags = p->q.flags;
			u.mve_flags = p->q.mve_flags;
			mve_queue_add(&s->events, input ? EV_INPUT : EV_OUTPUT,
				      sizeof(u), &u);
		}
		list_del(&p->node);
		kfree(p);
	}
	list_splice(&todo, list);
}

int mve_session_enqueue(struct mve_session *s, const struct mve_user_queue *q,
			bool input)
{
	struct pending *p;
	bool activated;
	int ret;

	mutex_lock(&s->lock);
	if (input)
		s->eos = q->flags & 1;
	else
		s->no_output_yet = false;
	ret = give_buffer(s, q, input);
	if (ret == MVE_TIMEOUT) {
		p = kmalloc(sizeof(*p), GFP_KERNEL);
		if (p) {
			p->q = *q;
			list_add_tail(&p->node, input ? &s->pending_in :
							&s->pending_out);
			ret = MVE_OK;
		} else {
			ret = MVE_NOMEM;
		}
	}
	if (!ret) {
		s->idle = NOT_IDLE;
		s->keep = true;
	}
	activated = s->activated;
	mutex_unlock(&s->lock);
	if (activated && schedule_session(s))
		mver_irq_signal_mve(s);
	return ret;
}

int mve_session_release_ref(struct mve_session *s, u64 handle)
{
	struct buffer *b;
	bool activated;

	mutex_lock(&s->lock);
	b = find_buffer(s, handle);
	if (b)
		request(s, REQ_RELEASE_REF, 4, &b->va);
	activated = s->activated;
	mutex_unlock(&s->lock);
	if (activated && schedule_session(s))
		mver_irq_signal_mve(s);
	return MVE_OK;
}

/* A buffer the firmware gives back; false if it is nobody's */
static void buffer_back(struct mve_session *s, u16 event)
{
	union {
		struct msg_frame f;
		struct msg_bitstream m;
		u32 w[18];
	} msg;
	bool input = event == EV_INPUT;
	struct buffer *b = NULL, *i;
	u16 code, size, id;

	if (!q_read(s, input ? &s->in : &s->out, &code, &size, &msg,
		    sizeof(msg)))
		return;
	if (input)
		s->n_input--;
	else
		s->n_output--;
	id = msg.w[0];
	list_for_each_entry(i, &s->buffers, node)
		if (i->id >= 0 && (u16)i->id == id) {
			b = i;
			break;
		}
	if (!b || b->port != !input) {
		WARN_ON_ONCE(1);
		return;
	}
	if (((s->flush & 2) && b->port) || ((s->flush & 1) && !b->port)) {
		/* a quick flush: given again when it ends */
		list_move_tail(&b->flush, &s->flush_list);
		b->id = -1;
		return;
	}
	if (--b->in_use < 0) {
		WARN_ON_ONCE(1);
		b->in_use = 0;
	}
	if (!b->in_use)
		b->id = -1;
	if (code == BUF_BITSTREAM) {
		b->user.filled = msg.m.filled;
		b->user.offset = msg.m.offset;
		b->user.flags = msg.m.flags & 0x4b1;
		b->user.timestamp = msg.m.tag;
	} else {
		b->user.flags = (msg.f.flags & 0x10000 ? 1 : 0) |
				(msg.f.flags & 0x4000 ? 4 : 0) |
				(msg.f.flags & 0x2000 ? 8 : 0) |
				(msg.f.flags & 0x8000 ? 0x200 : 0);
		b->user.mve_flags = msg.f.flags;
		if (msg.f.flags & 0xc)
			b->user.filled = 3 * msg.f.visible_height *
					 msg.f.visible_width / 2;
		b->user.pic_index = 0;
		b->user.timestamp = msg.f.tag;
		b->user.visible_width = msg.f.visible_width;
		b->user.visible_height = msg.f.visible_height;
	}
	if (!input && (b->user.flags & 1))
		s->eos = false;
	mve_queue_add(&s->events, event, sizeof(b->user), &b->user);
	give_pending(s, input);
}

/*
 * The firmware's calls to the host: memory of its own
 */
static void handle_rpc(struct mve_session *s)
{
	struct rpc *r = mve_rsrc_dma_mem_map(s->rpc);
	u32 id, n, max, va = 0, *pages, align;
	char text[256];

	if (ACCESS_ONCE(r->state) != RPC_CALL)
		return;
	rmb();
	id = r->id;
	switch (id) {
	case 1: /* print */
		memcpy(text, r->param, sizeof(text) - 1);
		text[sizeof(text) - 1] = 0;
		mve_queue_add(&s->events, 0, strlen(text) + 1, text);
		r->size = 0;
		break;
	case 2: /* size, max size, u8 frames, u8 log2 alignment */
		n = PAGE_ALIGN(r->param[0]) >> PAGE_SHIFT;
		max = PAGE_ALIGN(r->param[1]) >> PAGE_SHIFT;
		align = r->param[2] >> 8 & 0xff;
		pages = n && n <= 0x4000 && align < 28 ? alloc_mve_pages(n) :
							 NULL;
		if (pages) {
			va = mve_mmu_map_in(
				s->mmu,
				&mve_regions[r->param[2] & 0xff ?
						     MVE_REGION_FRAMES :
						     MVE_REGION_BITSTREAM],
				pages, n, max, 1u << align, true);
			if (!va)
				free_mve_pages(pages, n);
		}
		r->param[0] = va;
		r->size = 4;
		break;
	case 3: /* address, new size */
		if (!mve_mmu_resize(s->mmu, r->param[0],
				    PAGE_ALIGN(r->param[1]) >> PAGE_SHIFT))
			va = r->param[0];
		mver_scheduler_flush_tlb(s);
		r->param[0] = va;
		r->size = 4;
		break;
	case 4: /* address */
		if (r->param[0])
			mve_mmu_unmap(s->mmu, r->param[0]);
		mver_scheduler_flush_tlb(s);
		r->size = 0;
		break;
	default:
		pr_err("mve: firmware calls %u, unknown\n", id);
		r->size = 0;
		break;
	}
	wmb();
	r->state = id >= 1 && id <= 4 ? RPC_RETURN : 0;
	wmb();
	if (id != 1)
		mver_irq_signal_mve(s);
}

/* An input or output flush is done */
static void flushed(struct mve_session *s, u16 event)
{
	u32 port = event == EV_OUTPUT_FLUSHED;
	struct mve_user_queue q;
	struct buffer *b, *tmp;

	if (!s->flush) {
		/* buffers still out: the firmware is asked again */
		list_for_each_entry(b, &s->buffers, node)
			if (b->port == port && b->in_use > 0) {
				request(s,
					port ? REQ_OUTPUT_FLUSH :
					       REQ_INPUT_FLUSH,
					0, NULL);
				mver_irq_signal_mve(s);
				return;
			}
		mve_queue_add(&s->events, event, 0, NULL);
		return;
	}
	s->flushed |= port ? 2 : 1;
	if ((s->flush & 3) != s->flushed)
		return;
	list_for_each_entry_safe(b, tmp, &s->flush_list, flush) {
		list_del_init(&b->flush);
		if (--b->in_use < 0)
			b->in_use = 0;
		memset(&q, 0, sizeof(q));
		q.handle = b->user.handle;
		q.addr = b->user.addr;
		q.filled = b->user.filled;
		q.flags = s->flush & 8 ? b->user.flags | 1 :
					 b->user.flags & ~1u;
		q.mve_flags = b->user.mve_flags;
		if (give_buffer(s, &q, !b->port)) {
			WARN_ON_ONCE(1);
		} else {
			s->idle = NOT_IDLE;
			s->keep = true;
		}
	}
	s->flush = s->flushed = 0;
	if (s->state != GOING_OUT)
		answered(s);
}

static void firmware_idle(struct mve_session *s)
{
	if (s->idle == IDLE_1 && s->n_input && s->n_output) {
		s->idle = IDLE_2;
		mver_irq_signal_mve(s);
	} else if (s->idle == IDLE_1 || s->idle == IDLE_2) {
		s->idle = IDLE;
		s->keep = false;
	} else {
		s->idle = IDLE_1;
		mver_irq_signal_mve(s);
	}
}

/* Everything the firmware has said */
static void handle_messages(struct mve_session *s)
{
	u32 d[0x148 / 4], ev[3];
	u16 code, size;
	int was;

	while (q_read(s, &s->msg, &code, &size, d, sizeof(d))) {
		s->wd_pings = 0;
		s->wd_state = WD_FINE;
		switch (code) {
		case RSP_SWITCHED_IN:
			if (s->state != WAITING)
				answered(s);
			if (s->answers)
				mver_irq_signal_mve(s);
			mve_queue_add(&s->events, EV_SWITCHED_IN, size, d);
			break;
		case RSP_SWITCHED_OUT:
			was = s->state;
			set_state(s, OUT);
			s->out_wanted = false;
			if (was == WAITING)
				s->keep = true;
			mve_queue_add(&s->events, EV_SWITCHED_OUT, size, d);
			break;
		case RSP_OPTION_SET:
		case RSP_OPTION_FAILED:
			ev[0] = ~0u;
			ev[1] = code == RSP_OPTION_SET ? 0 : MVE_FIRMWARE;
			answered(s);
			mve_queue_add(&s->responses, EV_SET, 8, ev);
			break;
		case RSP_JOB_DEQUEUED:
			if (s->state <= GOING_IN)
				answered(s);
			break;
		case RSP_INPUT:
		case RSP_OUTPUT:
			if (s->state <= GOING_IN)
				answered(s);
			buffer_back(s, code == RSP_INPUT ? EV_INPUT :
							   EV_OUTPUT);
			break;
		case RSP_INPUT_FLUSHED:
			flushed(s, EV_INPUT_FLUSHED);
			break;
		case RSP_OUTPUT_FLUSHED:
			flushed(s, EV_OUTPUT_FLUSHED);
			break;
		case RSP_PONG:
			if (s->state <= GOING_IN)
				answered(s);
			mve_queue_add(&s->events, EV_PONG, 0, NULL);
			break;
		case RSP_ERROR:
			if (size > 84)
				size = 84;
			pr_err("mve: firmware error %u: %.*s\n", d[0],
			       max(size - 4, 0), (char *)&d[1]);
			if (d[0] == MVE_FIRMWARE && s->state == WAITING)
				mve_queue_interrupt(&s->responses);
			mve_queue_add(&s->events, EV_ERROR, size, d);
			break;
		case RSP_STATE_CHANGE:
			s->fw_state = s->fw_want = d[0];
			if (s->state != GOING_OUT)
				answered(s);
			mve_queue_add(&s->events, EV_STATE_CHANGED, 4, d);
			break;
		case RSP_DUMP:
			mve_queue_add(&s->events, EV_DUMP, 0, NULL);
			break;
		case RSP_IDLE:
			firmware_idle(s);
			break;
		case RSP_FRAME_ALLOC:
			mve_queue_add(&s->events, EV_FRAME_ALLOC, size, d);
			break;
		case RSP_SEQUENCE:
			mve_queue_add(&s->events, EV_SEQUENCE, size, d);
			break;
		case RSP_REF_UNUSED:
			mve_queue_add(&s->events, EV_REF_UNUSED, size, d);
			break;
		case BUF_PARAM:
			if (d[0] != 19) /* the DPB's, a decoder's */
				mve_queue_add(&s->events, EV_BUF_PARAM, size,
					      d);
			break;
		case RSP_EVENT:
			if (d[0] == 1 || d[0] == 2) {
				ev[0] = 0;
				ev[1] = 1;
				ev[2] = d[0] == 1 ? 0x8000100b : 0x80001006;
				mve_queue_add(&s->events, EV_STREAM_ERROR, 12,
					      ev);
			} else if (d[0] == 3 && size >= 4) {
				if (s->state <= GOING_IN)
					answered(s);
				mve_queue_add(&s->events, EV_PROCESSED,
					      size - 4, &d[1]);
			}
			/* 4 a reference, 5 tracing: nobody here asks */
			break;
		default:
			pr_err("mve: firmware message %u, unknown\n", code);
			break;
		}
	}
}

/*
 * mve_rsrc.ko's scheduler calls these, with its semaphore held
 */
static void irq_callback(void *cookie)
{
	struct mve_session *s = cookie;

	if (!get_session(s))
		return;
	mutex_lock(&s->lock);
	if (s->fw) {
		handle_rpc(s);
		handle_messages(s);
		s->wd_time = jiffies;
	}
	mutex_unlock(&s->lock);
	mve_session_put(s);
}

/* 0 nothing now, 1 idle, 2 work, 3 no session or idle and out, 4 wants out */
static int has_work(void *cookie)
{
	struct mve_session *s = cookie;
	int ret;

	if (!get_session(s))
		return 3;
	mutex_lock(&s->lock);
	if (s->state != OUT) {
		if (s->out_wanted)
			ret = 4;
		else
			ret = !s->answers && s->idle == IDLE;
	} else if (s->answers || s->keep || s->fw_state != s->fw_want) {
		ret = 2;
	} else if (s->idle == IDLE) {
		ret = 3;
	} else {
		ret = s->one_frame ? 2 : 0;
	}
	mutex_unlock(&s->lock);
	mve_session_put(s);
	return ret;
}

static void switchout_callback(void *cookie, int only_idle)
{
	struct mve_session *s = cookie;

	if (!get_session(s))
		return;
	mutex_lock(&s->lock);
	if (!only_idle || s->idle == IDLE)
		switch_out(s);
	mutex_unlock(&s->lock);
	mve_session_put(s);
}

static void switchin_callback(void *cookie)
{
	struct mve_session *s = cookie;
	struct {
		u16 cores, frames;
		u32 flags;
	} job = {.cores = mver_scheduler_get_ncores()};

	if (!get_session(s))
		return;
	mutex_lock(&s->lock);
	s->idle = NOT_IDLE;
	s->keep = false;
	s->one_frame = num_sessions > 1;
	job.frames = s->one_frame;
	if (request(s, REQ_JOB, sizeof(job), &job))
		goto out;
	if (s->fw_want != FW_NONE && s->fw_want != s->fw_state) {
		if (request(s, s->fw_want ? REQ_GO : REQ_STOP, 0, NULL))
			goto out;
		s->fw_want = FW_NONE;
	}
	if (s->state == OUT)
		set_state(s, GOING_IN);
	mver_irq_signal_mve(s);
out:
	mutex_unlock(&s->lock);
	mve_session_put(s);
}

static void switchout_completed_callback(void *cookie)
{
	struct mve_session *s = cookie;

	irq_callback(s);
	if (!get_session(s))
		return;
	mutex_lock(&s->lock);
	set_state(s, OUT);
	mutex_unlock(&s->lock);
	mve_session_put(s);
}

/* How many pictures wait, for the clock */
static int buffers_callback(void *cookie)
{
	struct mve_session *s = cookie;
	int n;

	if (!get_session(s))
		return -1;
	mutex_lock(&s->lock);
	n = s->n_input;
	if (n <= 1 && s->eos)
		n = 2;
	if (s->no_output_yet)
		n += 2;
	mutex_unlock(&s->lock);
	mve_session_put(s);
	return n;
}

/* A session that says nothing for 3 s is pinged, three times, then taken to
 * be dead */
static void watch_session(struct mve_session *s)
{
	struct buffer *b;
	bool hung = false;

	mutex_lock(&s->lock);
	if ((s->state == WAITING || s->state == RUNNING) &&
	    s->wd_state != WD_HUNG &&
	    time_after(jiffies, s->wd_time + msecs_to_jiffies(WD_PERIOD))) {
		if (s->wd_pings > 2) {
			/* out of the scheduler once: close does not do it again,
			 * and nothing schedules it any more */
			hung = s->registered;
			s->registered = false;
			s->wd_state = WD_HUNG;
			mve_queue_add(&s->events, EV_HUNG, 0, NULL);
			list_for_each_entry(b, &s->buffers, node)
				b->in_use = 0;
		} else {
			request(s, REQ_PING, 0, NULL);
			mver_irq_signal_mve(s);
			s->wd_state = WD_PINGED;
			s->wd_pings++;
		}
	}
	mutex_unlock(&s->lock);
	if (hung) {
		pr_err("mve: the firmware hangs\n");
		mver_scheduler_stop(s);
		mver_scheduler_unregister_session(s);
	}
}

static void watchdog(struct work_struct *work)
{
	struct mve_session *s, *prev = NULL;
	bool again;

	mutex_lock(&sessions_lock);
	list_for_each_entry(s, &sessions, node) {
		kref_get(&s->kref);
		mutex_unlock(&sessions_lock);
		if (prev)
			mve_session_put(prev);
		watch_session(s);
		prev = s;
		mutex_lock(&sessions_lock);
		if (list_empty(&s->node))
			break; /* destroyed: the rest the next time */
	}
	again = num_sessions > 0;
	mutex_unlock(&sessions_lock);
	if (prev)
		mve_session_put(prev);
	if (again)
		schedule_delayed_work(&watchdog_work,
				      msecs_to_jiffies(WD_PERIOD));
}

/*
 * Sessions
 */
static int queue_create(struct mve_session *s, struct queue *q, int region)
{
	u32 *pages;

	q->host = mve_rsrc_dma_mem_alloc(PAGE_SIZE, 0);
	q->mve = mve_rsrc_dma_mem_alloc(PAGE_SIZE, 0);
	if (!q->host || !q->mve)
		return -ENOMEM;
	pages = mve_rsrc_dma_mem_get_pages(q->host);
	if (!pages || mve_mmu_map(s->mmu, mve_regions[region].start, pages, 1,
				  PTE_SHARED_RW | PTE_RW))
		return -ENOMEM;
	pages = mve_rsrc_dma_mem_get_pages(q->mve);
	if (!pages || mve_mmu_map(s->mmu, mve_regions[region + 1].start, pages,
				  1, PTE_SHARED_RW | PTE_RW))
		return -ENOMEM;
	return 0;
}

/* the memory, before the page tables that point to it are gone: the hardware
 * has left the session by then */
static void queue_destroy(struct queue *q)
{
	if (q->host)
		mve_rsrc_dma_mem_free(q->host);
	if (q->mve)
		mve_rsrc_dma_mem_free(q->mve);
}

static void release_session(struct mve_session *s)
{
	struct buffer *b, *tmp;
	struct pending *p, *ptmp;

	list_for_each_entry_safe(b, tmp, &s->buffers, node)
		drop_buffer(s, b);
	list_for_each_entry_safe(p, ptmp, &s->pending_in, node)
		kfree(p);
	list_for_each_entry_safe(p, ptmp, &s->pending_out, node)
		kfree(p);
	mve_queue_clear(&s->events);
	mve_queue_clear(&s->responses);
	mve_fw_unload(s->mmu, s->fw);
	mve_mmu_destroy(s->mmu);
	queue_destroy(&s->msg);
	queue_destroy(&s->in);
	queue_destroy(&s->out);
	if (s->rpc)
		mve_rsrc_dma_mem_free(s->rpc);
	kfree(s);
}

static void free_session(struct kref *kref)
{
	release_session(container_of(kref, struct mve_session, kref));
}

int mve_session_create(struct file *file, const char *role)
{
	struct mve_session *s, *other = NULL;
	u32 *pages;

	if (strcmp(role, ROLE))
		return MVE_NOT_IMPLEMENTED;
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return MVE_NOMEM;
	kref_init(&s->kref);
	mutex_init(&s->lock);
	mutex_init(&s->config_lock);
	s->file = file;
	s->no_output_yet = true;
	s->next_id = 1;
	s->wd_time = jiffies;
	INIT_LIST_HEAD(&s->pending_in);
	INIT_LIST_HEAD(&s->pending_out);
	INIT_LIST_HEAD(&s->buffers);
	INIT_LIST_HEAD(&s->flush_list);
	/* bytes, what mve_base.ko's queues hold */
	mve_queue_init(&s->events, 4 * 0x4000);
	mve_queue_init(&s->responses, 4 * 0x2000);
	s->mmu = mve_mmu_create();
	if (!s->mmu || queue_create(s, &s->msg, MVE_REGION_MSG_HOST) ||
	    queue_create(s, &s->in, MVE_REGION_IN_HOST) ||
	    queue_create(s, &s->out, MVE_REGION_OUT_HOST))
		goto fail;
	s->rpc = mve_rsrc_dma_mem_alloc(PAGE_SIZE, 0);
	pages = s->rpc ? mve_rsrc_dma_mem_get_pages(s->rpc) : NULL;
	if (!pages || mve_mmu_map(s->mmu, mve_regions[MVE_REGION_RPC].start,
				  pages, 1, PTE_SHARED_RW | PTE_RW))
		goto fail;

	mver_pm_request_resume();
	mutex_lock(&sessions_lock);
	if (num_sessions == 1) {
		other = list_first_entry(&sessions, struct mve_session, node);
		kref_get(&other->kref);
	}
	list_add(&s->node, &sessions);
	if (++num_sessions == 1)
		schedule_delayed_work(&watchdog_work,
				      msecs_to_jiffies(WD_PERIOD));
	mutex_unlock(&sessions_lock);
	if (other) {
		/* so far it ran without end: out, to come back for a picture
		 * at a time */
		mutex_lock(&other->lock);
		other->keep = true;
		switch_out(other);
		mutex_unlock(&other->lock);
		mve_session_put(other);
	}
	return MVE_OK;
fail:
	if (s->mmu)
		release_session(s);
	else
		kfree(s);
	return MVE_NOMEM;
}

void mve_session_destroy(struct file *file)
{
	struct mve_session *s;
	bool registered;

	mutex_lock(&sessions_lock);
	list_for_each_entry(s, &sessions, node)
		if (s->file == file) {
			list_del_init(&s->node);
			num_sessions--;
			mutex_unlock(&sessions_lock);
			mutex_lock(&s->lock);
			registered = s->registered;
			s->registered = false;
			mutex_unlock(&s->lock);
			if (registered) {
				mver_scheduler_stop(s);
				mver_scheduler_unregister_session(s);
			}
			mver_pm_request_suspend();
			mve_session_put(s);
			return;
		}
	mutex_unlock(&sessions_lock);
}

int mve_session_activate(struct mve_session *s, u32 *version)
{
	int ncores = mver_scheduler_get_ncores();
	struct mve_fw *fw;
	u16 ver;

	mutex_lock(&s->lock);
	if (s->fw) {
		mutex_unlock(&s->lock);
		return MVE_NOT_READY;
	}
	fw = mve_fw_load(s->mmu, mve_dev, ncores, &ver);
	s->fw = fw;
	s->activated = fw;
	mutex_unlock(&s->lock);
	if (!fw)
		return MVE_NOT_READY;
	*version = ver;
	if (!mver_scheduler_register_session(
		    s, mve_mmu_ctrl(s->mmu), ncores, false, irq_callback,
		    has_work, switchout_callback, switchin_callback,
		    switchout_completed_callback, buffers_callback)) {
		/* as before activation, so that it can be tried again */
		mutex_lock(&s->lock);
		s->activated = false;
		s->fw = NULL;
		mve_fw_unload(s->mmu, fw);
		mutex_unlock(&s->lock);
		return MVE_NOT_READY;
	}
	mutex_lock(&s->lock);
	s->registered = true;
	mutex_unlock(&s->lock);
	return MVE_OK;
}

int mve_session_flush(struct mve_session *s, u32 flags)
{
	int ret = MVE_OK;

	if (!(flags & 3))
		return MVE_BAD_PARAM;
	mutex_lock(&s->lock);
	if (s->wd_state == WD_HUNG) {
		ret = MVE_HW;
		goto out;
	}
	if (s->fw)
		handle_messages(s);
	if (flags & 1) {
		ret = request(s, REQ_INPUT_FLUSH, 0, NULL);
		mver_irq_signal_mve(s);
		return_pending(s, &s->pending_in, true);
	}
	if (flags & 2) {
		ret |= request(s, REQ_OUTPUT_FLUSH, 0, NULL);
		mver_irq_signal_mve(s);
		return_pending(s, &s->pending_out, false);
	}
	if (!ret && (flags & 4)) {
		s->flush = flags;
		s->flushed = 0;
	}
out:
	mutex_unlock(&s->lock);
	if (!ret && s->activated)
		schedule_session(s);
	return ret;
}

int mve_session_set_state(struct mve_session *s, u32 state)
{
	int ret = MVE_OK;

	if (state != FW_STOPPED && state != FW_RUNNING)
		return MVE_BAD_PARAM;
	mutex_lock(&s->lock);
	if (s->fw)
		handle_messages(s);
	if (s->wd_state == WD_HUNG || s->fw_want == FW_NONE)
		ret = MVE_HW;
	else
		s->fw_want = state;
	mutex_unlock(&s->lock);
	if (!ret && s->activated)
		schedule_session(s);
	return ret;
}

struct mve_event *mve_session_event(struct mve_session *s, u32 timeout)
{
	return s->activated ? mve_queue_get(&s->events, timeout) : NULL;
}

/* A firmware message from user space: an option, which the firmware answers,
 * or a parameter in the input queue, which it does not */
static int config(struct mve_session *s, u32 code, const void *data, u32 size,
		  u32 *fw_error);

int mve_session_config(struct mve_session *s, const void *data, u32 size,
		       u32 *fw_error)
{
	u32 code;
	int ret;

	if (size < 4 || size > 0x300)
		return MVE_BAD_PARAM;
	memcpy(&code, data, 4);
	if (code != REQ_SET_OPTION && code != BUF_PARAM)
		return MVE_BAD_PARAM;
	mutex_lock(&s->config_lock);
	ret = config(s, code, data, size, fw_error);
	mutex_unlock(&s->config_lock);
	return ret;
}

static int config(struct mve_session *s, u32 code, const void *data, u32 size,
		  u32 *fw_error)
{
	struct mve_event *ev;
	u32 answer;
	int before, ret;

	if (s->wd_state == WD_HUNG)
		return MVE_HW;
	if (!s->activated || !schedule_session(s))
		return MVE_BUSY;
	mutex_lock(&s->lock);
	if (s->state == OUT || s->state == GOING_OUT) {
		ret = MVE_BUSY;
		goto out;
	}
	set_state(s, WAITING);
	/* an answer to a request whose wait was cut short */
	mve_queue_flush(&s->responses);
	before = s->answers;
	ret = code == REQ_SET_OPTION ?
		      request(s, code, size - 4, data + 4) :
		      q_write(s, &s->in, code, size - 4, data + 4);
	s->idle = NOT_IDLE;
	mver_irq_signal_mve(s);
	if (ret) {
		*fw_error = ret;
		ret = MVE_FIRMWARE;
		set_state(s, RUNNING);
		goto out;
	}
	if (s->answers == before) {
		if (s->state == WAITING)
			answered(s);
		goto out;
	}
	mutex_unlock(&s->lock);
	ev = mve_queue_get(&s->responses, 10000);
	mutex_lock(&s->lock);
	if (!ev) {
		ret = s->responses.interrupted ? MVE_HW : MVE_BUSY;
		set_state(s, RUNNING);
		goto out;
	}
	if (ev->e.code != EV_SET || ev->e.size != 8) {
		ret = MVE_HW;
	} else {
		memcpy(&answer, ev->e.data + 4, 4);
		if (answer) {
			*fw_error = answer;
			ret = MVE_FIRMWARE;
		}
	}
	kfree(ev);
	if (s->state == WAITING)
		answered(s);
out:
	mutex_unlock(&s->lock);
	return ret;
}

unsigned int mve_session_poll(struct file *file, poll_table *wait)
{
	struct mve_session *s = mve_session_get(file);
	unsigned int mask = 0;

	if (!s)
		return 0;
	poll_wait(file, &s->events.wait, wait);
	if (!list_empty(&s->events.list))
		mask = POLLIN | POLLRDNORM;
	mve_session_put(s);
	return mask;
}

void mve_sessions_init(struct device *dev)
{
	mve_dev = dev;
}

/* no file is open, so no session is left */
void mve_sessions_exit(void)
{
	cancel_delayed_work_sync(&watchdog_work);
}
