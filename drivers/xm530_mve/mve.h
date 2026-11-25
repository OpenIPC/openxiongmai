/* SPDX-License-Identifier: GPL-2.0 */
/*
 * xm530_mve: the parts of the driver. See xm530_mve.c for what it is.
 */
#ifndef XM530_MVE_H
#define XM530_MVE_H

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/wait.h>

/*
 * mve_rsrc.ko, the vendor's scheduler, register and memory driver. It is
 * given a session's address only as a cookie. Memory is in pages named by
 * their physical address, which is their DMA address on this SoC. The "track"
 * arguments name the caller for its allocation list.
 */
extern u32 mve_rsrc_mem_alloc_page_track(const char *func, int line);
extern void mve_rsrc_mem_free_page_track(u32 page, const char *func, int line);
extern u32 *mve_rsrc_mem_alloc_pages_track(u32 n, const char *func, int line);
extern void mve_rsrc_mem_free_pages_track(u32 *pages, u32 n, const char *func,
					  int line);
extern u32 mve_rsrc_mem_read32(u32 addr);
extern void mve_rsrc_mem_write32(u32 addr, u32 val);
extern void mve_rsrc_mem_clean_cache_range(u32 addr, u32 size);
extern void *mve_rsrc_mem_cpu_map_page(u32 page);

struct mve_dma;
extern struct mve_dma *mve_rsrc_dma_mem_alloc(u32 size, int type);
extern void mve_rsrc_dma_mem_free(struct mve_dma *mem);
extern void *mve_rsrc_dma_mem_map(struct mve_dma *mem);
extern u32 *mve_rsrc_dma_mem_get_pages(struct mve_dma *mem);

extern int mver_scheduler_register_session(
	void *session, u32 mmu_ctrl, int ncores, bool secure,
	void (*irq)(void *), int (*has_work)(void *),
	void (*switchout)(void *, int), void (*switchin)(void *),
	void (*switchout_completed)(void *), int (*buffers)(void *));
extern void mver_scheduler_unregister_session(void *session);
/* runs the irq callback of the session itself: no session lock held */
extern int mver_scheduler_execute(void *session);
extern void mver_scheduler_stop(void *session);
extern void mver_scheduler_flush_tlb(void *session);
extern int mver_scheduler_get_ncores(void);
extern void mver_irq_signal_mve(void *session);
extern int mver_pm_request_resume(void);
extern int mver_pm_request_suspend(void);
extern void mver_dvfs_request_max_frequency(void);
extern u32 mver_reg_get_fuse(void);
extern u32 mver_reg_get_version(void);

#define alloc_mve_page() mve_rsrc_mem_alloc_page_track(__func__, __LINE__)
#define free_mve_page(p) mve_rsrc_mem_free_page_track(p, __func__, __LINE__)
#define alloc_mve_pages(n) mve_rsrc_mem_alloc_pages_track(n, __func__, __LINE__)
#define free_mve_pages(p, n) \
	mve_rsrc_mem_free_pages_track(p, n, __func__, __LINE__)

/* The results of a command, the first word of its reply */
enum mve_status {
	MVE_OK = 0,
	MVE_NOT_READY = 1,
	MVE_BAD_PARAM = 2,
	MVE_BAD_PORT = 3,
	MVE_FIRMWARE = 4, /* the second word says what */
	MVE_HW = 5,
	MVE_NOMEM = 6,
	MVE_NOT_IMPLEMENTED = 7,
	MVE_BUSY = 8,
	MVE_TIMEOUT = 9, /* also: the firmware's queue is full */
	MVE_NOT_ACTIVE = 10,
};

/* The firmware's virtual address space */
enum mve_region {
	MVE_REGION_FW0 = 0,
	MVE_REGION_MSG_HOST,
	MVE_REGION_MSG_MVE,
	MVE_REGION_IN_HOST,
	MVE_REGION_IN_MVE,
	MVE_REGION_OUT_HOST,
	MVE_REGION_OUT_MVE,
	MVE_REGION_RPC,
	MVE_REGION_BITSTREAM,
	MVE_REGION_FRAMES,
	MVE_REGION_FW1, /* .. FW7 */
	MVE_REGIONS = MVE_REGION_FW1 + 7,
};
struct mve_range {
	u32 start, end;
};
extern const struct mve_range mve_regions[MVE_REGIONS];

/* Page table entries: bits 30-31 attributes, 2-21 the page, 0-1 access */
#define PTE_RO 1u
#define PTE_EXEC 2u
#define PTE_RW 3u
#define PTE_SHARED_RW (3u << 30)
#define PTE(page, flags) ((((page) >> 12) << 2) | (flags))

struct mve_mmu;
struct mve_mmu *mve_mmu_create(void);
void mve_mmu_destroy(struct mve_mmu *mmu);
u32 mve_mmu_ctrl(struct mve_mmu *mmu);
/* maps n pages at va; `flags` goes into every entry */
int mve_mmu_map(struct mve_mmu *mmu, u32 va, const u32 *pages, u32 n,
		u32 flags);
/* copies a table of 1024 entries over the 4 MiB at va */
int mve_mmu_map_table(struct mve_mmu *mmu, u32 va, const u32 *entries);
/* finds max free pages in the range, maps n pages there and keeps the rest
 * reserved; `own`: the pages become the MMU's to free. 0 if it cannot. */
u32 mve_mmu_map_in(struct mve_mmu *mmu, const struct mve_range *range,
		   u32 *pages, u32 n, u32 max, u32 align, bool own);
int mve_mmu_resize(struct mve_mmu *mmu, u32 va, u32 n);
void mve_mmu_unmap(struct mve_mmu *mmu, u32 va);

struct mve_fw;
struct mve_fw *mve_fw_load(struct mve_mmu *mmu, struct device *dev, int ncores,
			   u16 *version);
void mve_fw_unload(struct mve_mmu *mmu, struct mve_fw *fw);

/* Events, as MVE_BASE_GET_EVENT returns them */
struct mve_event {
	struct list_head node;
	struct {
		u16 code, size;
		u8 data[];
	} e __aligned(4);
};
struct mve_event_queue {
	struct list_head list;
	struct mutex lock;
	wait_queue_head_t wait;
	u32 bytes, max;
	bool interrupted;
};
void mve_queue_init(struct mve_event_queue *q, u32 max);
/* _clear: nobody else has the queue any more */
void mve_queue_clear(struct mve_event_queue *q);
void mve_queue_flush(struct mve_event_queue *q);
/* copies the event in; lost when the queue is full, as in mve_base.ko */
void mve_queue_add(struct mve_event_queue *q, u16 code, u16 size,
		   const void *data);
/* NULL on a timeout (ms, 0 does not wait), an interrupt or a signal */
struct mve_event *mve_queue_get(struct mve_event_queue *q, u32 timeout);
void mve_queue_interrupt(struct mve_event_queue *q);

/* User space's buffer, 120 bytes */
struct mve_user_buffer {
	u64 timestamp;
	u64 handle;
	u64 addr;
	u64 crc_addr;
	u32 allocator;
	u32 size;
	u32 width, height, stride, stride_align;
	u32 format;
	u32 visible_width, visible_height;
	u32 hint0, hint1; /* 68, 72: handed back as registered */
	u32 filled, offset, flags, mve_flags;
	u32 pic_index;
	u16 crop_left, crop_top;
	u8 depth, pad[3];
	u32 crc_allocator, crc_size, crc_offset;
	u32 pad2;
};
/* MVE_BASE_FILL_THIS_BUFFER / EMPTY_THIS_BUFFER, 208 bytes */
struct mve_user_queue {
	u64 handle;
	u64 addr;
	u32 filled, flags, mve_flags, crc_offset;
	u64 timestamp;
	u8 nroi, pad[3];
	u8 roi[16][10];
	u32 pad2;
};

struct mve_session;
int mve_session_create(struct file *file, const char *role);
void mve_session_destroy(struct file *file);
void mve_sessions_init(struct device *dev);
void mve_sessions_exit(void);
struct mve_session *mve_session_get(struct file *file);
void mve_session_put(struct mve_session *s);
int mve_session_activate(struct mve_session *s, u32 *version);
int mve_session_flush(struct mve_session *s, u32 flags);
int mve_session_set_state(struct mve_session *s, u32 state);
struct mve_event *mve_session_event(struct mve_session *s, u32 timeout);
int mve_session_config(struct mve_session *s, const void *data, u32 size,
		       u32 *fw_error);
int mve_session_register(struct mve_session *s, u32 port,
			 const struct mve_user_buffer *b);
int mve_session_unregister(struct mve_session *s, u64 handle);
int mve_session_enqueue(struct mve_session *s, const struct mve_user_queue *q,
			bool input);
int mve_session_release_ref(struct mve_session *s, u64 handle);
unsigned int mve_session_poll(struct file *file, poll_table *wait);

#endif
