// SPDX-License-Identifier: MIT
/*
 * xmispd: an open ISP daemon for the XM510, a drop-in for the vendor
 * /usr/bin/isp that the streamer starts. It brings VI, the sensor and the
 * ISP up through libxmisp, prints the vendor daemon's ready line, and waits;
 * on SIGTERM or SIGINT it takes the vendor daemon's way out.
 *
 * Each frame start runs auto-exposure and auto white balance. With
 * XMISPD_VERBOSE set in its environment it prints every change (majestic
 * logs each line, so not by default).
 *
 * Anti-flicker follows majestic's isp.antiFlicker ("disabled", "50" or
 * "60"; see /api/v1/config.schema.json), read from /etc/majestic.yaml at
 * start and again on SIGHUP. XMISPD_ANTIFLICKER in the environment (0, 50
 * or 60) overrides it.
 *
 * Day and night follow majestic's automatic night mode, whose keys it
 * reads the same way: with nightMode.lightMonitor true (and no
 * lightSensorPin or min/maxThreshold, which are majestic's own sources),
 * the picture goes black and white once AE has run out of exposure and gain
 * for nightMode.autoNightDelay seconds (default 15), or the gain has
 * reached nightMode.autoNightGain, and back to colour once the gain stays
 * at or below nightMode.autoDayGain (default 2) for autoDayDelay seconds
 * (default 60). On the XM510 majestic cannot see the exposure, which this
 * daemon holds.
 *
 * The sensor mode follows the vendor daemon's own configuration, which
 * majestic writes from its video0.size: "Profile=960" in ISP_CONFIG prefers
 * a sensor's 960P script (1280x960). The script that started is reported as
 * the vendor daemon reports its sensor, "<name> sensor init OK", the name
 * carrying the mode ("SC1235 960p"), which is how majestic sizes the main
 * stream.
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

#include "daynight.h"
#include "xmisp.h"

#define LOCK_FILE "/tmp/Isplock"
#define MAJESTIC_YAML "/etc/majestic.yaml"
/* The vendor daemon's configuration, at the path its binary fixes. */
#define ISP_CONFIG "/mnt/mtd/Config/Json/format"

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

/* "50" or "60" (quoted or not) as Hz, "disabled" or absent as 0, anything
 * else as -1. */
static int hz_of(const char *v) {
  while (*v == ' ' || *v == '"' || *v == '\'')
    v++;
  if (!strncmp(v, "50", 2))
    return 50;
  if (!strncmp(v, "60", 2))
    return 60;
  if (!*v || !strncmp(v, "disabled", 8) || !strncmp(v, "0", 1))
    return 0;
  return -1;
}

/* A key of majestic's YAML, indented under a top-level section: its value
 * (spaces and quotes trimmed) into val, 1 if present, 0 if not. */
static int yaml_get(const char *section, const char *key, char *val,
                    size_t len) {
  FILE *f = fopen(MAJESTIC_YAML, "r");
  if (!f)
    return 0;
  char line[256];
  const size_t sl = strlen(section), kl = strlen(key);
  int in = 0, found = 0;
  while (!found && fgets(line, sizeof(line), f)) {
    line[strcspn(line, "\r\n#")] = 0;
    if (line[0] && line[0] != ' ') {
      in = !strncmp(line, section, sl) && line[sl] == ':';
      continue;
    }
    const char *k = line + strspn(line, " ");
    if (in && !strncmp(k, key, kl) && k[kl] == ':') {
      const char *v = k + kl + 1;
      v += strspn(v, " \"'");
      snprintf(val, len, "%s", v);
      val[strcspn(val, "\"' ")] = 0;
      found = 1;
    }
  }
  fclose(f);
  return found;
}

/* isp.antiFlicker. */
static int antiflicker_hz(void) {
  const char *env = getenv("XMISPD_ANTIFLICKER");
  if (env)
    return hz_of(env);
  char v[32];
  return yaml_get("isp", "antiFlicker", v, sizeof(v)) ? hz_of(v) : 0;
}

/* A nightMode number, or def if absent or empty. */
static float night_num(const char *key, float def) {
  char v[32];
  if (!yaml_get("nightMode", key, v, sizeof(v)) || !v[0])
    return def;
  return strtof(v, NULL);
}

/* Day and night from nightMode, at start and on SIGHUP. */
static void daynight(struct xmisp *isp) {
  char v[32];
  const int on =
      yaml_get("nightMode", "lightMonitor", v, sizeof(v)) && !strcmp(v, "true");
  /* majestic's other sources take precedence over the exposure */
  const int other =
      (yaml_get("nightMode", "lightSensorPin", v, sizeof(v)) && v[0]) ||
      (yaml_get("nightMode", "minThreshold", v, sizeof(v)) && v[0] &&
       yaml_get("nightMode", "maxThreshold", v, sizeof(v)) && v[0]);
  if (!on || other) {
    if (xmisp_set_daynight(isp, NULL) < 0)
      error("%s", xmisp_error(isp));
    return;
  }
  const struct xmisp_dn_cfg cfg = {
      night_num("autoNightGain", 0.0f),
      night_num("autoDayGain", 2.0f),
      (uint32_t)(night_num("autoNightDelay", 15.0f) * 1000.0f),
      (uint32_t)(night_num("autoDayDelay", 60.0f) * 1000.0f),
  };
  if (xmisp_set_daynight(isp, &cfg) < 0)
    error("%s", xmisp_error(isp));
  else
    debug("day/night: black and white once %s for %.0f s, colour at %.1fx "
          "or less for %.0f s",
          cfg.night_gain > 0 ? "the gain reaches its night value"
                             : "the exposure runs out",
          cfg.night_ms / 1000.0f, cfg.day_gain, cfg.day_ms / 1000.0f);
}

static void antiflicker(struct xmisp *isp) {
  const int hz = antiflicker_hz();
  if (hz < 0)
    error("isp.antiFlicker: not disabled, 50 or 60; left off");
  if (xmisp_set_antiflicker(isp, hz > 0 ? (unsigned)hz : 0) < 0)
    error("%s", xmisp_error(isp));
  else if (hz > 0)
    debug("anti-flicker: %d Hz mains, exposure in whole %.2f ms periods", hz,
          500.0 / hz);
}

/* The picture's lines ISP_CONFIG asks for: 960 for "Profile=960", else 0
 * (the sensor's first script). */
static unsigned profile_height(void) {
  FILE *f = fopen(ISP_CONFIG, "r");
  if (!f)
    return 0;
  char line[128];
  unsigned h = 0;
  while (fgets(line, sizeof(line), f))
    if (!strncmp(line, "Profile=", 8) && atoi(line + 8) == 960)
      h = 960;
  fclose(f);
  return h;
}

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
  sigaddset(&set, SIGHUP);
  sigprocmask(SIG_BLOCK, &set, NULL);

  struct xmisp *isp = xmisp_open();
  if (!isp) {
    error("cannot open the ISP devices: %s", strerror(errno));
    return 1;
  }
  if (xmisp_start_mode(isp, xmisp_scripts, profile_height()) < 0) {
    error("%s", xmisp_error(isp));
    xmisp_close(isp);
    return 1;
  }
  debug("%s sensor init OK", xmisp_script(isp)->name);
  debug("%s up: open ISP, %s, %s", xmisp_script(isp)->name,
        xmisp_ae_enabled(isp) ? "auto-exposure"
                              : "fixed exposure (no driver for the sensor)",
        xmisp_awb_enabled(isp)
            ? "auto white balance"
            : "fixed white balance (no range for the sensor)");
  const int verbose = getenv("XMISPD_VERBOSE") != NULL;

  const int fd = xmisp_fd(isp);
  if (fcntl(fd, F_SETOWN, getpid()) < 0 ||
      fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_ASYNC) < 0)
    error("no frame signal: %s", strerror(errno));

  antiflicker(isp);
  daynight(isp);

  /* The vendor daemon's last start-up line; the streamer starts video once
   * it has it. */
  debug("ispMsgReceive start!");

  unsigned long frames = 0;
  int failing = 0; /* AE errors are logged when they start and stop */
  int night = 0;
  for (;;) {
    siginfo_t si;
    const int sig = sigwaitinfo(&set, &si);
    if (sig == SIGIO) {
      frames++;
      const int r = xmisp_frame(isp);
      if (r < 0 && !failing)
        error("3A: %s; logged again when it recovers", xmisp_error(isp));
      else if (r >= 0 && failing)
        debug("3A recovered after frame %lu", frames - 1);
      failing = r < 0;
      if (xmisp_night(isp) != night) {
        night = xmisp_night(isp);
        float mean, gain;
        uint32_t lines;
        xmisp_exposure(isp, &mean, &lines, &gain);
        debug("%s (gain %.1f, mean %.1f)",
              night ? "night: black and white" : "day: colour", gain, mean);
      }
      if (r > 0 && verbose) {
        float mean, gain;
        uint32_t lines;
        xmisp_exposure(isp, &mean, &lines, &gain);
        uint32_t grey;
        float cb, cr, wr, wb;
        xmisp_white_balance(isp, &grey, &cb, &cr, &wr, &wb);
        debug("frame %lu: mean %.1f -> %u lines, gain %.2f, NR %.2f; %u grey, "
              "Cb %.1f Cr %.1f -> R %.2f B %.2f",
              frames, mean, (unsigned)lines, gain, xmisp_nr_strength(isp),
              (unsigned)grey, cb, cr, wr, wb);
      }
    } else if (sig == SIGHUP) {
      antiflicker(isp);
      daynight(isp);
    } else if (sig == SIGTERM || sig == SIGINT)
      break;
  }

  xmisp_stop(isp);
  xmisp_close(isp);
  debug("open ISP stopped after %lu frames", frames);
  return 0;
}
