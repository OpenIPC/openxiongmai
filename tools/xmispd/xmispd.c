// SPDX-License-Identifier: MIT
/*
 * xmispd: an open ISP daemon for the XM510, a drop-in for the vendor
 * /usr/bin/isp that the streamer starts. It brings VI, the sensor and the
 * ISP up through libxmisp, prints the vendor daemon's ready line, and waits;
 * on SIGTERM or SIGINT it takes the vendor daemon's way out.
 *
 * M1: exposure and white balance stay at the script's fixed state.
 *
 * Output follows the vendor daemon's format, which the streamer forwards to
 * its log: "\033[0;32;32mDEBUG: ..." or "\033[0;32;31mERR: ...", each line
 * followed by "\033[m".
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

#include "xmisp.h"

#define LOCK_FILE "/tmp/Isplock"

static void say(const char *kind, const char *fmt, ...) {
  va_list ap;
  printf("\033[0;32;%sm%s: ", kind[0] == 'E' ? "31" : "32", kind);
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n\033[m");
  fflush(stdout);
}

#define debug(...) say("DEBUG", __VA_ARGS__)
#define error(...) say("ERR", __VA_ARGS__)

int main(void) {
  /* One daemon at a time, as the vendor's lock file has it. */
  const int lock = open(LOCK_FILE, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
  if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB) < 0) {
    error("%s: %s", LOCK_FILE,
          errno == EWOULDBLOCK ? "another ISP daemon runs" : strerror(errno));
    return 1;
  }

  /* Signals are taken by sigwaitinfo() below, not in handlers. */
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGIO);
  sigaddset(&set, SIGTERM);
  sigaddset(&set, SIGINT);
  sigprocmask(SIG_BLOCK, &set, NULL);

  struct xmisp *isp = xmisp_open();
  if (!isp) {
    error("cannot open the ISP devices: %s", strerror(errno));
    return 1;
  }
  if (xmisp_start(isp, xmisp_scripts) < 0) {
    error("%s", xmisp_error(isp));
    xmisp_close(isp);
    return 1;
  }
  debug("%s up: open ISP, fixed exposure and white balance",
        xmisp_script(isp)->name);

  const int fd = xmisp_fd(isp);
  if (fcntl(fd, F_SETOWN, getpid()) < 0 ||
      fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_ASYNC) < 0)
    error("no frame signal: %s", strerror(errno));

  /* The vendor daemon's last start-up line; the streamer starts video once
   * it has it. */
  debug("ispMsgReceive start!");

  unsigned long frames = 0;
  for (;;) {
    siginfo_t si;
    const int sig = sigwaitinfo(&set, &si);
    if (sig == SIGIO)
      frames++;
    else if (sig == SIGTERM || sig == SIGINT)
      break;
  }

  xmisp_stop(isp);
  xmisp_close(isp);
  debug("open ISP stopped after %lu frames", frames);
  return 0;
}
