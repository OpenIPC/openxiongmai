// SPDX-License-Identifier: MIT
/*
 * xm-vi-hang: reproduce the XM530/XM550 VI driver hang without a camera
 * application, and test ways around it.
 *
 * A VI init (_IO('V', 6) on /dev/vi) freezes the whole board now and then
 * (the CPU stops with interrupts off; only a watchdog or a power cycle gets
 * out) when the process before it left VI capturing with ext channel 1 in
 * use. One run of this tool:
 *  1. resets MMZ and inits VI, as an SDK start does (VI takes its channel
 *     buffers from MMZ when a channel is set up, which fails with EPERM
 *     unless MMZ was reset);
 *  2. replays the ISP and MIPI writes an SDK start makes after VI init
 *     (isp_writes.h), without which capture stays stopped;
 *  3. sets VI up as that start does: main channel 0 and ext channel 1;
 *  4. takes frames from ext channel 1 for a few seconds;
 *  5. exits, leaving VI capturing.
 * Run it in a loop: the next run's VI init is the call that may freeze.
 * On an XM550 + IMX335 that was about one run in 16.
 *
 * Needs the sensor running: start the camera application once after boot,
 * then stop it. Expect to power-cycle the board: unless a watchdog is
 * armed, nothing resets it from this hang.
 *
 * Usage: xm-vi-hang [-t seconds] [-0] [-i] [-M] [-n] [-N] [-p] [-S] [-x]
 *                   [-B command] [-A command]
 *   -t  take frames for this long (default 3)
 *   -0  set up channel 0 only (no ext channel 1)
 *   -i  VI init only: no ISP writes, no setup, no frames
 *   -M  no MMZ reset before the VI init
 *   -n  enable ext channel 1 but take no frames from it
 *   -p  disable the channels before the VI init (the run before must have
 *       left them enabled)
 *   -x  disable the channels before exiting
 *   -N  turn 3D noise reduction off before the VI init, and give its DMA a
 *       few frames to stop: the way around the hang
 *   -S  unmap the channels' buffers before the VI init. VI init forgets
 *       them without unmapping, which leaks about 7 MB of vmalloc a run; a
 *       channel setup with a size VI refuses unmaps them first.
 * A run that fails a step exits 1, so `while xm-vi-hang; do ...; done`
 * stops rather than counting it.
 *   -B  run this shell command right before the VI init
 *   -A  run this shell command right after it (say, to put the sensor into
 *       standby around the init and wake it again)
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* /dev/vi, as the vendor SDK calls it on XM530/XM550 */
#define VI_SET_DEV 0x40105600u   /* x, y, width, height */
#define VI_SET_CHN 0x400c5601u   /* channel, width, height */
#define VI_ENABLE 0x40045602u    /* channel */
#define VI_DISABLE 0x40045603u   /* channel */
#define VI_GET_FRAME 0xc0205604u /* 32-byte frame record */
#define VI_RELEASE 0x40205605u
#define VI_INIT 0x5606u
#define VI_SET_RATE 0x400c5607u /* channel, 24, 0x01ffffff (as recorded) */
#define MMZ_RESET 0x4d00u       /* /dev/mmz */
#define ISP_WRITE 0x400c4901u   /* /dev/isp: address, value, 0 */
/* 3D noise reduction control: it reads and writes reference frames in DDR
 * for as long as it is on. VI init resets it with the rest of the ISP, and
 * a reset in the middle of its DMA is what freezes the bus. */
#define NR_3D_CTRL 0x30040000u

static const uint32_t isp_writes[][2] = {
#include "isp_writes.h"
};

static double now(void) {
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

static void say(const char *msg) {
  printf("xm-vi-hang %.3f %s\n", now(), msg);
  fflush(stdout);
}

/* The SoC has one core: a line written to a pipe right before a call that
 * freezes never reaches whoever reads the pipe. Give it time to leave. */
static void settle(void) { usleep(300000); }

/* A run that couldn't do what it was asked to exits non-zero, so a loop
 * stops instead of counting it */
static int fail(const char *what) {
  char msg[96];

  snprintf(msg, sizeof(msg), "FAILED: %s (%m)", what);
  say(msg);
  return 1;
}

/* Ext channel 1 (unless main_only), then channel 0 */
static int disable_channels(int vi, int main_only) {
  uint32_t c1 = 1, c0 = 0;

  if (!main_only && ioctl(vi, VI_DISABLE, &c1))
    return fail("disable channel 1");
  if (ioctl(vi, VI_DISABLE, &c0))
    return fail("disable channel 0");
  return 0;
}

int main(int argc, char **argv) {
  int secs = 3, mmz_reset = 1, init_only = 0, no_frames = 0, pre = 0, post = 0,
      main_only = 0, nr_off = 0, shrink = 0, c;
  const char *before = NULL, *after = NULL;
  uint32_t dev[4] = {0, 0, 1920, 1080}, c0[3] = {0, 1920, 1080};
  uint32_t c1[3] = {1, 704, 576}, e0 = 0, e1 = 1;
  uint32_t r0[3] = {0, 24, 0x01ffffff}, r1[3] = {1, 24, 0x01ffffff};
  long frames = 0;
  char msg[96];

  while ((c = getopt(argc, argv, "t:0iMnNpSxB:A:")) != -1)
    switch (c) {
    case 't':
      secs = atoi(optarg);
      break;
    case '0':
      main_only = 1;
      break;
    case 'B':
      before = optarg;
      break;
    case 'A':
      after = optarg;
      break;
    case 'i':
      init_only = 1;
      break;
    case 'M':
      mmz_reset = 0;
      break;
    case 'n':
      no_frames = 1;
      break;
    case 'N':
      nr_off = 1;
      break;
    case 'S':
      shrink = 1;
      break;
    case 'p':
      pre = 1;
      break;
    case 'x':
      post = 1;
      break;
    default:
      fprintf(stderr,
              "usage: %s [-t seconds] [-0] [-i] [-M] [-n] [-N] [-p] [-S] [-x] "
              "[-B command] [-A command]\n",
              argv[0]);
      return 2;
    }

  int vi = open("/dev/vi", O_RDONLY), isp = open("/dev/isp", O_RDWR);
  int mmz = mmz_reset ? open("/dev/mmz", O_RDWR) : -2;
  if (vi < 0 || isp < 0 || mmz == -1) {
    perror("open");
    return 1;
  }
  if (pre) {
    if (disable_channels(vi, main_only))
      return 1;
    say("channels disabled");
  }
  if (nr_off) {
    uint32_t a[3] = {NR_3D_CTRL, 0, 0};

    if (ioctl(isp, ISP_WRITE, a))
      return fail("3D-NR off");
    usleep(200000); /* three frames at 15 fps */
    say("3D-NR off");
  }
  if (shrink) {
    /* A channel setup unmaps the channel's old buffer mappings before it
     * checks the new size, so one with a size VI refuses ("too small")
     * unmaps them and stops there. The channels are disabled first, so
     * nothing uses the mappings until the init clears them. */
    uint32_t s1[3] = {1, 64, 64}, s0[3] = {0, 64, 64};

    if (disable_channels(vi, main_only))
      return 1;
    uint32_t *sizes[2] = {s0, s1};
    for (int chn = main_only ? 0 : 1; chn >= 0; chn--) {
      char what[48];

      /* Refused as too small is the expected outcome; anything else may
       * have left the buffers mapped */
      if (ioctl(vi, VI_SET_CHN, sizes[chn]) == 0 || errno != EPERM) {
        snprintf(what, sizeof(what), "unmap channel %d buffers", chn);
        return fail(what);
      }
    }
    say("channel buffers unmapped");
  }
  if (before && system(before) != 0)
    return fail("-B command");
  say("vi init next");
  settle();
  if (mmz_reset && ioctl(mmz, MMZ_RESET, 0))
    return fail("MMZ reset");
  if (ioctl(vi, VI_INIT, 0))
    return fail("VI init");
  say("vi init done");
  if (after && system(after) != 0)
    return fail("-A command");
  if (init_only)
    return 0;

  for (unsigned i = 0; i < sizeof(isp_writes) / sizeof(isp_writes[0]); i++) {
    uint32_t a[3] = {isp_writes[i][0], isp_writes[i][1], 0};

    if (ioctl(isp, ISP_WRITE, a)) {
      snprintf(msg, sizeof(msg), "ISP write %u of %u (0x%08x)", i + 1,
               (unsigned)(sizeof(isp_writes) / sizeof(isp_writes[0])),
               isp_writes[i][0]);
      return fail(msg);
    }
  }
  struct {
    unsigned long req;
    void *arg;
    const char *what;
  } setup[] = {
      {VI_SET_DEV, dev, "device"},  {VI_SET_CHN, c0, "channel 0"},
      {VI_ENABLE, &e0, "enable 0"}, {VI_SET_CHN, c1, "channel 1"},
      {VI_ENABLE, &e1, "enable 1"}, {VI_SET_RATE, r0, "rate 0"},
      {VI_SET_RATE, r1, "rate 1"},
  };
  /* -0: device, channel 0, its enable and its rate */
  static const unsigned all[] = {0, 1, 2, 3, 4, 5, 6}, main0[] = {0, 1, 2, 5};
  const unsigned *steps = main_only ? main0 : all;
  unsigned nsteps = main_only ? 4 : 7;
  for (unsigned k = 0; k < nsteps; k++) {
    unsigned i = steps[k];
    if (ioctl(vi, setup[i].req, setup[i].arg)) {
      snprintf(msg, sizeof(msg), "VI setup: %s", setup[i].what);
      return fail(msg);
    }
  }

  for (double end = now() + secs; now() < end;) {
    uint32_t rec[8];

    if (no_frames) {
      usleep(100000);
      continue;
    }
    memset(rec, 0, sizeof(rec));
    rec[0] = main_only ? 0 : 1;
    /* A get fails while no frame is ready yet */
    if (ioctl(vi, VI_GET_FRAME, rec) == 0 && rec[2]) {
      if (ioctl(vi, VI_RELEASE, rec))
        return fail("frame release");
      frames++;
    } else {
      usleep(5000);
    }
  }
  if (!no_frames && !frames)
    return fail("no frame came");
  if (post && disable_channels(vi, main_only))
    return 1;
  snprintf(msg, sizeof(msg), "%ld channel %d frames; %s", frames,
           main_only ? 0 : 1,
           post ? "channels disabled, exiting" : "exiting, VI capturing");
  say(msg);
  return 0;
}
