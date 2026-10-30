/*
 * The sensor mode from majestic's main stream size: video0.size in
 * /etc/majestic.yaml. majestic passes "-r 0" (720P) whatever its
 * configuration, so without this every sensor with more than one mode runs
 * its default one, and the picture majestic encodes may come from a mode
 * of another size.
 */
#include <stdio.h>
#include <string.h>

#include "video_size.h"

#define MAJESTIC_YAML "/etc/majestic.yaml"

/* video0.size, "WxH", from majestic's configuration. 0 if found. */
static int video0_size(const char *path, unsigned *w, unsigned *h) {
  FILE *f = fopen(path, "r");
  if (!f)
    return -1;
  char line[256];
  int in_video0 = 0, found = -1;
  while (fgets(line, sizeof(line), f)) {
    const char *p = line;
    while (*p == ' ' || *p == '\t')
      p++;
    if (*p == '#' || *p == '\n' || *p == '\r' || !*p)
      continue;
    if (p == line) { /* a top-level key starts a section */
      in_video0 = !strncmp(p, "video0:", 7);
      continue;
    }
    if (in_video0 && !strncmp(p, "size:", 5) &&
        sscanf(p + 5, " %ux%u", w, h) == 2) {
      found = 0;
      break;
    }
  }
  fclose(f);
  return found;
}

/* The sensor mode for a picture of w x h: by its lines, as the modes are
 * named (720P, 960P, 1080P, 3M, 4M, 5M, 4K). */
static int profile_of(unsigned w, unsigned h) {
  (void)w;
  switch (h) {
  case 720:
    return P720_;
  case 960:
    return P960_;
  case 1080:
    return P1080_;
  case 1296: /* 2304x1296 */
  case 1536: /* 2048x1536 */
    return P1536_;
  case 1440: /* 2560x1440 */
  case 1520: /* 2688x1520 */
    return P4M_;
  case 1920: /* 2560x1920 */
  case 1944: /* 2592x1944 */
    return P5M_;
  case 2160:
    return P4K_;
  default:
    return -1;
  }
}

int video_size_profile(void) {
  unsigned w = 0, h = 0;
  if (video0_size(MAJESTIC_YAML, &w, &h) < 0)
    return -1;
  static const char *const names[] = {"720P", "1080P", "960P", "3M",
                                      "4M",   "5M",    "4K"};
  const int p = profile_of(w, h);
  printf("video0.size %ux%u: sensor mode %s\n", w, h,
         p < 0 ? "the sensor's default" : names[p]);
  return p;
}
