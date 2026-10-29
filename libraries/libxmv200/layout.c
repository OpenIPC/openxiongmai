// SPDX-License-Identifier: MIT
/*
 * A board layout as text, for a configuration file. One item per line,
 * '#' starts a comment:
 *
 *   chip xm535d1                 (optional: the running kernel's)
 *   fps 12
 *   sensor SC2336 bus=0 addr=0x30 x0=1 width=1920 height=1080
 *   sensor SC2336 bus=0 addr=0x32 x0=1 width=1920 height=1080 phy=1 vi_idx=1
 *   stream 1920x1080 sensors=0,1   (the two spliced top to bottom)
 *   stream 640x360 sensors=0,1
 *
 * A sensor's model "auto" probes its chip ID. Its keys: bus, addr, flip,
 * x0, y0, width, height, phy, vi_dev, vi_idx, mipi, ldc (struct
 * xmv200_layout_sensor). Streams are VENC channels 0, 1, ... in order.
 */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xmv200.h"

static char *word(char **p) {
  char *s = *p, *w;

  while (*s == ' ' || *s == '\t')
    s++;
  if (!*s)
    return NULL;
  w = s;
  while (*s && *s != ' ' && *s != '\t')
    s++;
  if (*s)
    *s++ = 0;
  *p = s;
  return w;
}

static int sensor_key(struct xmv200_layout_sensor *s, const char *k,
                      uint32_t v) {
  static const struct {
    const char *k;
    size_t off;
  } u32[] = {
      {"x0", offsetof(struct xmv200_layout_sensor, x0)},
      {"y0", offsetof(struct xmv200_layout_sensor, y0)},
      {"width", offsetof(struct xmv200_layout_sensor, width)},
      {"height", offsetof(struct xmv200_layout_sensor, height)},
      {"phy", offsetof(struct xmv200_layout_sensor, phy)},
      {"vi_dev", offsetof(struct xmv200_layout_sensor, vi_dev)},
      {"vi_idx", offsetof(struct xmv200_layout_sensor, vi_idx)},
      {"mipi", offsetof(struct xmv200_layout_sensor, mipi)},
  };

  if (!strcmp(k, "bus"))
    s->bus = v;
  else if (!strcmp(k, "addr"))
    s->addr = v;
  else if (!strcmp(k, "flip"))
    s->flip = v;
  else if (!strcmp(k, "ldc"))
    s->ldc = v;
  else {
    for (unsigned i = 0; i < sizeof(u32) / sizeof(u32[0]); i++)
      if (!strcmp(k, u32[i].k)) {
        *(uint32_t *)((char *)s + u32[i].off) = v;
        return 0;
      }
    return -1;
  }
  return 0;
}

int xmv200_layout_parse(const char *text, struct xmv200_layout *l) {
  /* the model and chip names point into this copy, kept for the layout */
  char *buf = strdup(text), *line, *next;
  int n = 0;

  if (!buf)
    return -1;
  memset(l, 0, sizeof(*l));
  for (line = buf; line; line = next) {
    char *p, *w;

    n++;
    next = strchr(line, '\n');
    if (next)
      *next++ = 0;
    if ((p = strchr(line, '#')) != NULL)
      *p = 0;
    p = line;
    if (!(w = word(&p)))
      continue;
    if (!strcmp(w, "chip")) {
      l->chip = word(&p);
    } else if (!strcmp(w, "fps")) {
      w = word(&p);
      l->fps = w ? strtoul(w, NULL, 0) : 0;
    } else if (!strcmp(w, "sensor")) {
      struct xmv200_layout_sensor *s;

      if (l->n_sensors >= XMV200_MAX_SENSORS || !(w = word(&p)))
        goto bad;
      s = &l->sensor[l->n_sensors++];
      s->model = strcmp(w, "auto") ? w : NULL;
      while ((w = word(&p)) != NULL) {
        char *eq = strchr(w, '=');

        if (!eq)
          goto bad;
        *eq = 0;
        if (sensor_key(s, w, strtoul(eq + 1, NULL, 0)))
          goto bad;
      }
    } else if (!strcmp(w, "stream")) {
      struct xmv200_layout_stream *st;
      char *x;

      if (l->n_streams >= XMV200_MAX_STREAMS || !(w = word(&p)) ||
          !(x = strchr(w, 'x')))
        goto bad;
      st = &l->stream[l->n_streams++];
      st->width = strtoul(w, NULL, 0);
      st->height = strtoul(x + 1, NULL, 0);
      st->sensors = 1;
      while ((w = word(&p)) != NULL) {
        if (strncmp(w, "sensors=", 8))
          goto bad;
        st->sensors = 0;
        for (char *c = w + 8; *c; c++)
          if (*c >= '0' && *c <= '9')
            st->sensors |= 1u << (*c - '0');
      }
    } else {
      goto bad;
    }
  }
  return 0;
bad:
  fprintf(stderr, "libxmv200: layout line %d: not understood\n", n);
  free(buf);
  return -1;
}

int xmv200_layout_format(const struct xmv200_layout *l, char *out, int cap) {
  int n = 0;

#define PUT(...)                                                              \
  n += snprintf(out + (n < cap ? n : cap), n < cap ? cap - n : 0, __VA_ARGS__)
  if (l->chip)
    PUT("chip %s\n", l->chip);
  PUT("fps %u\n", l->fps);
  for (int c = 0; c < l->n_sensors; c++) {
    const struct xmv200_layout_sensor *s = &l->sensor[c];

    PUT("sensor %s bus=%d addr=0x%02x", s->model ? s->model : "auto", s->bus,
        s->addr);
    if (s->flip)
      PUT(" flip=0x%02x", s->flip);
    PUT(" x0=%u y0=%u", s->x0, s->y0);
    if (s->width)
      PUT(" width=%u height=%u", s->width, s->height);
    if (s->phy)
      PUT(" phy=%u", s->phy);
    if (s->vi_dev)
      PUT(" vi_dev=%u", s->vi_dev);
    if (s->vi_idx)
      PUT(" vi_idx=%u", s->vi_idx);
    if (s->mipi)
      PUT(" mipi=%u", s->mipi);
    if (s->ldc)
      PUT(" ldc=1");
    PUT("\n");
  }
  for (int i = 0; i < l->n_streams; i++) {
    PUT("stream %ux%u sensors=", l->stream[i].width, l->stream[i].height);
    for (int c = 0, first = 1; c < XMV200_MAX_SENSORS; c++)
      if (l->stream[i].sensors & 1u << c) {
        PUT("%s%d", first ? "" : ",", c);
        first = 0;
      }
    PUT("\n");
  }
#undef PUT
  return n;
}
