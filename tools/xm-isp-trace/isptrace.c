// SPDX-License-Identifier: MIT
/*
 * isptrace.so: an LD_PRELOAD tracer for a closed ISP daemon, such as the
 * XM510 vendor /usr/bin/isp. It records what the daemon does to the
 * hardware: every /dev/isp register write and read, every other ioctl with
 * its argument before and after, the files it opens, its xmap() mappings,
 * its message queue traffic and reads, and the frame-start SIGIO, so each
 * record carries the frame it belongs to. decode.py turns the binary log
 * into text.
 *
 * ISPTRACE names the output file (a FIFO works, for streaming the log off
 * the board); ISPTRACE_MAX caps its size in bytes (0 or unset: no cap).
 *
 * The record format is in isptrace.h.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <time.h>
#include <ucontext.h>
#include <unistd.h>

#include "isptrace.h"

#define BUF_SIZE (64 * 1024)
#define PAYLOAD_MAX 4096 /* a record's payload, truncated beyond */
#define FLUSH_WAIT_MS 1000
#define MAX_FDS 256

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned char buf[BUF_SIZE];
static size_t used;
static int out_fd = -1;
static unsigned long long out_max, out_total;
static int probe[2] = {-1, -1};
static pthread_mutex_t probe_lock = PTHREAD_MUTEX_INITIALIZER;
static char isp_fd[MAX_FDS];
static char mem_fd[MAX_FDS];

/* Set by the SIGIO trampoline, which can't take the lock. */
static volatile uint32_t frame;
static volatile uint32_t frame_us;
static uint32_t frame_logged;
static uint32_t flushed_us;

static int (*real_open)(const char *, int, ...);
static int (*real_ioctl)(int, int, ...);
static ssize_t (*real_read)(int, void *, size_t);
static int (*real_close)(int);
static int (*real_creat)(const char *, mode_t);
static FILE *(*real_fopen)(const char *, const char *);
static int (*real_access)(const char *, int);
static int (*real_msgsnd)(int, const void *, size_t, int);
static ssize_t (*real_msgrcv)(int, void *, size_t, long, int);

static void (*app_sigio)(int);
/* The daemon's other handlers (SIGINT, SIGTERM: its exit path). */
static void (*app_handler[NSIG])(int);
/* Set once one of them runs: every record is written out at once, since the
 * process may be gone before the next flush. */
static volatile int sync_out;

static uint32_t now_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)ts.tv_sec * 1000000u + (uint32_t)(ts.tv_nsec / 1000);
}

/* Called with signals blocked. The output is non-blocking: a reader that
 * falls behind for FLUSH_WAIT_MS, or goes away, ends the trace, and the
 * daemon runs on untraced. */
static void flush_locked(void) {
  size_t off = 0;
  int waited = 0;
  while (out_fd >= 0 && off < used) {
    const ssize_t n = write(out_fd, buf + off, used - off);
    if (n > 0) {
      off += (size_t)n;
      continue;
    }
    if (n < 0 && errno == EINTR)
      continue;
    if (n < 0 && errno == EAGAIN && waited < FLUSH_WAIT_MS) {
      struct pollfd pfd = {.fd = out_fd, .events = POLLOUT};
      poll(&pfd, 1, 10);
      waited += 10;
      continue;
    }
    if (n < 0 && errno == EPIPE) {
      /* The write raised SIGPIPE for this thread; take it while it is
       * blocked, so it never reaches the daemon. */
      sigset_t pipe_sig;
      sigemptyset(&pipe_sig);
      sigaddset(&pipe_sig, SIGPIPE);
      const struct timespec now = {0, 0};
      sigtimedwait(&pipe_sig, NULL, &now);
    }
    real_close(out_fd);
    out_fd = -1;
  }
  used = 0;
}

static void put_locked(uint8_t type, uint8_t fd, uint32_t aux, uint16_t aux2,
                       uint32_t t_us, const void *p1, size_t n1, const void *p2,
                       size_t n2) {
  if (n1 > PAYLOAD_MAX)
    n1 = PAYLOAD_MAX, n2 = 0;
  else if (n1 + n2 > PAYLOAD_MAX)
    n2 = PAYLOAD_MAX - n1;
  const size_t len = n1 + n2;
  const size_t total = sizeof(struct isptrace_rec) + ((len + 3) & ~(size_t)3);
  if (out_fd < 0)
    return;
  if (out_max && out_total + total > out_max) {
    flush_locked();
    real_close(out_fd);
    out_fd = -1;
    return;
  }
  if (used + total > BUF_SIZE)
    flush_locked();
  struct isptrace_rec r = {
      .t_us = t_us,
      .aux = aux,
      .frame = (uint16_t)frame,
      .type = type,
      .fd = fd,
      .len = (uint16_t)len,
      .aux2 = aux2,
  };
  memcpy(buf + used, &r, sizeof(r));
  if (n1)
    memcpy(buf + used + sizeof(r), p1, n1);
  if (n2)
    memcpy(buf + used + sizeof(r) + n1, p2, n2);
  memset(buf + used + sizeof(r) + len, 0, total - sizeof(r) - len);
  used += total;
  out_total += total;
}

/* The vendor daemon runs its 3A in the SIGIO handler, whose ioctls come back
 * here: the locks are taken with signals blocked, so a handler never waits
 * on a lock the code it interrupted holds. */
static void lock_nosig(pthread_mutex_t *m, sigset_t *old) {
  sigset_t all;
  sigfillset(&all);
  pthread_sigmask(SIG_SETMASK, &all, old);
  pthread_mutex_lock(m);
}

static void unlock_nosig(pthread_mutex_t *m, const sigset_t *old) {
  pthread_mutex_unlock(m);
  pthread_sigmask(SIG_SETMASK, old, NULL);
}

/* Logs the frame starts since the last record, then the record. */
static void emit(uint8_t type, int fd, uint32_t aux, uint16_t aux2,
                 const void *p1, size_t n1, const void *p2, size_t n2) {
  const int err = errno;
  const uint32_t t = now_us();
  sigset_t old;
  lock_nosig(&lock, &old);
  const uint32_t f = frame;
  if (f != frame_logged) {
    put_locked(ISPTRACE_FRAME, 0, f, 0, frame_us, NULL, 0, NULL, 0);
    frame_logged = f;
  }
  put_locked(type, (uint8_t)fd, aux, aux2, t, p1, n1, p2, n2);
  /* At most 100 ms of records lost if the daemon is killed. */
  if (sync_out || t - flushed_us > 100000) {
    flush_locked();
    flushed_us = t;
  }
  unlock_nosig(&lock, &old);
  errno = err;
}

/* Copies n bytes from p if the whole range is readable; a pipe write fails
 * with EFAULT on a bad address instead of faulting, and the kernel here
 * (3.0) predates process_vm_readv(). */
static int safe_copy(void *dst, const void *p, size_t n) {
  if (!p || probe[0] < 0)
    return 0;
  sigset_t old;
  lock_nosig(&probe_lock, &old);
  int ok = write(probe[1], p, n) == (ssize_t)n &&
           real_read(probe[0], dst, n) == (ssize_t)n;
  if (!ok) {
    /* A fault part way leaves the bytes before it in the pipe. */
    static unsigned char junk[64];
    while (real_read(probe[0], junk, sizeof(junk)) > 0)
      ;
  }
  unlock_nosig(&probe_lock, &old);
  return ok;
}

static void sig_tramp(int sig) {
  sync_out = 1;
  emit(ISPTRACE_SIGNAL, 0, (uint32_t)sig, 2, NULL, 0, NULL, 0);
  if (app_handler[sig])
    app_handler[sig](sig);
}

static void sigio_tramp(int sig) {
  frame_us = now_us();
  frame++;
  if (app_sigio)
    app_sigio(sig);
}

/* Run on the first call, not as a constructor: musl runs constructors from
 * __libc_start_main, which a uClibc binary on libwrapper never calls. */
static pthread_once_t once = PTHREAD_ONCE_INIT;

static void init(void) {
  real_open = dlsym(RTLD_NEXT, "open");
  real_ioctl = dlsym(RTLD_NEXT, "ioctl");
  real_read = dlsym(RTLD_NEXT, "read");
  real_close = dlsym(RTLD_NEXT, "close");
  real_creat = dlsym(RTLD_NEXT, "creat");
  real_fopen = dlsym(RTLD_NEXT, "fopen");
  real_access = dlsym(RTLD_NEXT, "access");
  real_msgsnd = dlsym(RTLD_NEXT, "msgsnd");
  real_msgrcv = dlsym(RTLD_NEXT, "msgrcv");

  const char *path = getenv("ISPTRACE");
  const char *max = getenv("ISPTRACE_MAX");
  if (max)
    out_max = strtoull(max, NULL, 0);
  if (!path)
    return;
  out_fd = real_open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (out_fd < 0)
    return;
  if (pipe(probe) == 0) {
    for (int i = 0; i < 2; i++) {
      fcntl(probe[i], F_SETFD, FD_CLOEXEC);
      fcntl(probe[i], F_SETFL, O_NONBLOCK);
    }
  }
  fcntl(out_fd, F_SETFL, fcntl(out_fd, F_GETFL) | O_NONBLOCK);
  const struct isptrace_hdr h = {
      .magic = ISPTRACE_MAGIC,
      .version = ISPTRACE_VERSION,
      .t0_us = now_us(),
  };
  /* Written with the first records, by flush_locked(). */
  memcpy(buf, &h, sizeof(h));
  used = sizeof(h);
}

/* Like the constructor, only if libc runs it. */
__attribute__((destructor)) static void fini(void) {
  sigset_t old;
  lock_nosig(&lock, &old);
  flush_locked();
  unlock_nosig(&lock, &old);
}

static void track(int fd, const char *path) {
  if (fd >= 0 && fd < MAX_FDS) {
    isp_fd[fd] = path && !strcmp(path, "/dev/isp");
    mem_fd[fd] = path && !strcmp(path, "/dev/mem");
  }
}

int open(const char *path, int flags, ...) {
  pthread_once(&once, init);
  mode_t mode = 0;
  if (flags & O_CREAT) {
    va_list ap;
    va_start(ap, flags);
    mode = va_arg(ap, mode_t);
    va_end(ap);
  }
  const int fd = real_open(path, flags, mode);
  track(fd, path);
  emit(ISPTRACE_OPEN, fd, (uint32_t)flags, (uint16_t)fd, path, strlen(path) + 1,
       NULL, 0);
  return fd;
}

int creat(const char *path, mode_t mode) {
  pthread_once(&once, init);
  const int fd = real_creat(path, mode);
  track(fd, path);
  emit(ISPTRACE_CREAT, fd, mode, (uint16_t)fd, path, strlen(path) + 1, NULL, 0);
  return fd;
}

FILE *fopen(const char *path, const char *mode) {
  pthread_once(&once, init);
  FILE *f = real_fopen(path, mode);
  emit(ISPTRACE_FOPEN, f ? fileno(f) : -1, 0, f != NULL, path, strlen(path) + 1,
       mode, strlen(mode) + 1);
  return f;
}

int access(const char *path, int how) {
  pthread_once(&once, init);
  const int r = real_access(path, how);
  emit(ISPTRACE_ACCESS, 0, (uint32_t)how, (uint16_t)r, path, strlen(path) + 1,
       NULL, 0);
  return r;
}

int close(int fd) {
  pthread_once(&once, init);
  if (fd >= 0 && fd < MAX_FDS && fd != out_fd && fd != probe[0] &&
      fd != probe[1]) {
    isp_fd[fd] = 0;
    mem_fd[fd] = 0;
    emit(ISPTRACE_CLOSE, fd, 0, 0, NULL, 0, NULL, 0);
  }
  return real_close(fd);
}

/* The size of the argument an ioctl takes, or 0 if it is not encoded. */
static size_t ioc_size(unsigned cmd) {
  return _IOC_DIR(cmd) != _IOC_NONE ? _IOC_SIZE(cmd) : 0;
}

int ioctl(int fd, int req, ...) {
  pthread_once(&once, init);
  va_list ap;
  va_start(ap, req);
  void *arg = va_arg(ap, void *);
  va_end(ap);
  const unsigned cmd = (unsigned)req;

  if (fd >= 0 && fd < MAX_FDS && isp_fd[fd] &&
      (cmd == ISPTRACE_ISP_WRITE || cmd == ISPTRACE_ISP_READ)) {
    uint32_t rec[2] = {0, 0};
    safe_copy(rec, arg, sizeof(rec));
    const int r = real_ioctl(fd, req, arg);
    if (cmd == ISPTRACE_ISP_READ)
      safe_copy(rec, arg, sizeof(rec));
    emit(cmd == ISPTRACE_ISP_WRITE ? ISPTRACE_ISPW : ISPTRACE_ISPR, fd, rec[0],
         (uint16_t)r, &rec[1], 4, NULL, 0);
    return r;
  }

  /* An unencoded command may take a plain integer, not a pointer: the value
   * goes in the log, and the bytes it points at if they can be read. */
  size_t n = ioc_size(cmd);
  if (!n)
    n = 32;
  if (n > ISPTRACE_IOCTL_MAX)
    n = ISPTRACE_IOCTL_MAX;
  /* Off the stack: the vendor daemon's threads run on musl's small default
   * stacks, with little to spare. */
  static __thread struct {
    uint32_t arg;
    uint32_t valid;
    unsigned char b[ISPTRACE_IOCTL_MAX];
  } before;
  static __thread unsigned char after[ISPTRACE_IOCTL_MAX];
  before.arg = (uint32_t)(uintptr_t)arg;
  before.valid = safe_copy(before.b, arg, n) ? ISPTRACE_BEFORE : 0;
  const int r = real_ioctl(fd, req, arg);
  const int err = errno;
  const int ok = before.valid && safe_copy(after, arg, n);
  if (ok)
    before.valid |= ISPTRACE_AFTER;
  emit(ISPTRACE_IOCTL, fd, cmd, (uint16_t)(r < 0 ? -err : r), &before,
       8 + (before.valid ? n : 0), after, ok ? n : 0);
  errno = err;
  return r;
}

ssize_t read(int fd, void *p, size_t n) {
  pthread_once(&once, init);
  const ssize_t r = real_read(fd, p, n);
  if (fd != probe[0])
    emit(ISPTRACE_READ, fd, (uint32_t)n, (uint16_t)r, p,
         r > 0 ? (r > 64 ? 64 : (size_t)r) : 0, NULL, 0);
  return r;
}

int msgsnd(int qid, const void *p, size_t n, int flags) {
  pthread_once(&once, init);
  const int r = real_msgsnd(qid, p, n, flags);
  const size_t len = sizeof(long) + n;
  emit(ISPTRACE_MSGSND, 0, (uint32_t)qid, (uint16_t)r, p, len > 256 ? 256 : len,
       NULL, 0);
  return r;
}

ssize_t msgrcv(int qid, void *p, size_t n, long type, int flags) {
  pthread_once(&once, init);
  const ssize_t r = real_msgrcv(qid, p, n, type, flags);
  const size_t len = r >= 0 ? sizeof(long) + (size_t)r : 0;
  emit(ISPTRACE_MSGRCV, 0, (uint32_t)qid, (uint16_t)r, p, len > 256 ? 256 : len,
       NULL, 0);
  return r;
}

/* Register pages mapped through /dev/mem are handed out read-only: a store
 * faults, the handler logs it, performs it through a writable alias of the
 * page and steps over the instruction. The XM510 daemon programs the system
 * controller this way. Stores it can't decode (Thumb, multiple) make the
 * mapping writable from then on, and are logged once as such. */
#define MAX_TRAPS 16
#define REG_SPACE_END 0x80000000u /* DDR starts here */

/* The daemon's own SIGSEGV action, which faults outside the traced pages
 * go to; signal() records a later one here instead of replacing ours. */
static struct sigaction app_segv = {.sa_handler = SIG_DFL};
static volatile int segv_installed;

static struct trap {
  uintptr_t ro;
  unsigned char *rw;
  size_t len;
  uint32_t phys;
} traps[MAX_TRAPS];
static pthread_mutex_t trap_lock = PTHREAD_MUTEX_INITIALIZER;
static int (*real_munmap)(void *, size_t);

static struct trap *trap_at(uintptr_t a) {
  for (int i = 0; i < MAX_TRAPS; i++)
    if (traps[i].len && a - traps[i].ro < traps[i].len)
      return &traps[i];
  return NULL;
}

static void give_up(struct trap *t, uint32_t insn) {
  mprotect((void *)t->ro, t->len, PROT_READ | PROT_WRITE);
  emit(ISPTRACE_MEMRW, 0, t->phys, 0, &insn, 4, NULL, 0);
}

static void segv(int sig, siginfo_t *si, void *ucv) {
  ucontext_t *uc = ucv;
  struct sigcontext *m = &uc->uc_mcontext;
  unsigned long *r = &m->arm_r0; /* r0-r10, fp, ip, sp, lr, pc */
  const uintptr_t a = (uintptr_t)si->si_addr;
  struct trap *t = trap_at(a);
  if (!t) {
    /* Not ours: the daemon's handler, or the fault again without ours. */
    if (app_segv.sa_flags & SA_SIGINFO) {
      app_segv.sa_sigaction(sig, si, ucv);
    } else if (app_segv.sa_handler != SIG_DFL &&
               app_segv.sa_handler != SIG_IGN) {
      app_segv.sa_handler(sig);
    } else {
      struct sigaction dfl = {.sa_handler = SIG_DFL};
      sigemptyset(&dfl.sa_mask);
      sigaction(SIGSEGV, &dfl, NULL);
    }
    return;
  }
  if (m->arm_cpsr & 0x20) {
    give_up(t, 0);
    return;
  }
  const uint32_t insn = *(const uint32_t *)m->arm_pc;
  unsigned size, off;
  if ((insn & 0x0c100000) == 0x04000000) { /* STR, STRB */
    size = insn & (1u << 22) ? 1 : 4;
    if (insn & (1u << 25)) {
      const uint32_t rm = (uint32_t)r[insn & 15];
      const unsigned type = (insn >> 5) & 3, imm = (insn >> 7) & 31;
      if (type == 0)
        off = rm << imm;
      else if (type == 1)
        off = imm ? rm >> imm : 0;
      else if (type == 2)
        off = (uint32_t)((int32_t)rm >> (imm ? imm : 31));
      else {
        give_up(t, insn);
        return;
      }
    } else {
      off = insn & 0xfff;
    }
  } else if ((insn & 0x0e1000f0) == 0x000000b0) { /* STRH */
    size = 2;
    off = insn & (1u << 22) ? ((insn >> 4) & 0xf0) | (insn & 0xf)
                            : (uint32_t)r[insn & 15];
  } else {
    give_up(t, insn);
    return;
  }
  const unsigned rd = (insn >> 12) & 15, rn = (insn >> 16) & 15;
  if (rd == 15 || rn == 15) {
    give_up(t, insn);
    return;
  }
  const uint32_t v = (uint32_t)r[rd];
  unsigned char *w = t->rw + (a - t->ro);
  if (size == 4)
    *(volatile uint32_t *)w = v;
  else if (size == 2)
    *(volatile uint16_t *)w = (uint16_t)v;
  else
    *(volatile uint8_t *)w = (uint8_t)v;
  emit(ISPTRACE_MEMW, 0, t->phys + (uint32_t)(a - t->ro), (uint16_t)size, &v, 4,
       NULL, 0);
  /* Post-indexed, or pre-indexed with writeback: the base moves. */
  if (!(insn & (1u << 24)) || (insn & (1u << 21)))
    r[rn] += insn & (1u << 23) ? off : -off;
  m->arm_pc += 4;
}

static void trap_init(void) {
  real_munmap = dlsym(RTLD_NEXT, "munmap");
  struct sigaction sa = {.sa_sigaction = segv, .sa_flags = SA_SIGINFO};
  sigemptyset(&sa.sa_mask);
  sigaction(SIGSEGV, &sa, &app_segv);
  segv_installed = 1;
}

static pthread_once_t trap_once = PTHREAD_ONCE_INIT;

/* libwrapper's mmap for the uClibc binaries, which dlsym can't find from
 * here: it is mmap() at a 32-bit offset, so this does it. */
void *xmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
  pthread_once(&once, init);
  const int trap = out_fd >= 0 && fd >= 0 && fd < MAX_FDS && mem_fd[fd] &&
                   off < REG_SPACE_END && (prot & PROT_WRITE) &&
                   (flags & MAP_SHARED);
  void *r;
  if (trap) {
    pthread_once(&trap_once, trap_init);
    sigset_t old;
    lock_nosig(&trap_lock, &old);
    struct trap *t = NULL;
    for (int i = 0; i < MAX_TRAPS && !t; i++)
      if (!traps[i].len)
        t = &traps[i];
    /* The alias goes wherever the kernel puts it. */
    void *rw = t ? mmap(NULL, len, prot, flags & ~MAP_FIXED, fd, (off_t)off)
                 : MAP_FAILED;
    r = mmap(start, len, rw != MAP_FAILED ? prot & ~PROT_WRITE : prot, flags,
             fd, (off_t)off);
    if (rw != MAP_FAILED && r != MAP_FAILED) {
      t->rw = rw;
      t->phys = off;
      t->ro = (uintptr_t)r;
      t->len = len;
    } else if (rw != MAP_FAILED) {
      real_munmap(rw, len);
    }
    unlock_nosig(&trap_lock, &old);
  } else {
    r = mmap(start, len, prot, flags, fd, (off_t)off);
  }
  const uint32_t a[4] = {(uint32_t)len, (uint32_t)prot, (uint32_t)flags,
                         (uint32_t)(uintptr_t)r};
  emit(ISPTRACE_XMAP, fd, off, trap, a, sizeof(a), NULL, 0);
  return r;
}

int munmap(void *p, size_t len) {
  pthread_once(&once, init);
  if (!real_munmap)
    real_munmap = dlsym(RTLD_NEXT, "munmap");
  const uintptr_t a = (uintptr_t)p;
  sigset_t old;
  lock_nosig(&trap_lock, &old);
  for (int i = 0; i < MAX_TRAPS; i++) {
    struct trap *t = &traps[i];
    if (!t->len || a >= t->ro + t->len || a + len <= t->ro)
      continue;
    /* Part of a traced page left mapped is made writable, untraced. */
    if (a > t->ro || a + len < t->ro + t->len)
      give_up(t, ISPTRACE_PARTIAL_UNMAP);
    real_munmap(t->rw, t->len);
    t->len = 0;
  }
  unlock_nosig(&trap_lock, &old);
  return real_munmap(p, len);
}

/* signal() as musl has it (BSD semantics), on sigaction(), so the SIGIO
 * handler can be wrapped. */
void (*signal(int sig, void (*h)(int)))(int) {
  pthread_once(&once, init);
  if (sig == SIGSEGV && segv_installed) {
    void (*prev)(int) =
        app_segv.sa_flags & SA_SIGINFO ? SIG_DFL : app_segv.sa_handler;
    app_segv.sa_handler = h;
    app_segv.sa_flags = 0;
    emit(ISPTRACE_SIGNAL, 0, (uint32_t)sig, 1, NULL, 0, NULL, 0);
    return prev;
  }
  const int fn = h != SIG_DFL && h != SIG_IGN && h != SIG_ERR;
  const int wrap = fn && sig > 0 && sig < NSIG && sig != SIGSEGV;
  struct sigaction sa = {.sa_flags = SA_RESTART}, old;
  sa.sa_handler = !wrap ? h : sig == SIGIO ? sigio_tramp : sig_tramp;
  sigemptyset(&sa.sa_mask);
  void (*prev)(int) = NULL;
  if (sig > 0 && sig < NSIG)
    prev = sig == SIGIO ? app_sigio : app_handler[sig];
  if (wrap) {
    if (sig == SIGIO)
      app_sigio = h;
    else
      app_handler[sig] = h;
  }
  emit(ISPTRACE_SIGNAL, 0, (uint32_t)sig, wrap, NULL, 0, NULL, 0);
  if (sigaction(sig, &sa, &old) < 0)
    return SIG_ERR;
  if ((old.sa_handler == sigio_tramp || old.sa_handler == sig_tramp) && prev)
    return prev;
  return old.sa_handler;
}
