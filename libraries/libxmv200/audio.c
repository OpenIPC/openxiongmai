// SPDX-License-Identifier: MIT
/*
 * Audio on the V200 series: the codec's input and output as 16-bit mono
 * PCM through i2s.ko (/dev/ai, /dev/ao; open_i2s.ko is the same), its
 * volume, mute and gain through acodec.ko (/dev/acodec), and G.711.
 *
 * The sample rate is i2s.ko's: on the XM535D1/D2 an ioctl on /dev/ai sets
 * it (0x40044110), elsewhere the module's rate= does at load (8000 by
 * default, as the vendor's). Reads block until as many bytes as asked
 * for are there; writes until there is room.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "xmv200.h"

#define AI_SET_RATE 0x40044110
#define AO_GET_USED 0x80044105
#define AC_SET_VOLUME 0x40044101
#define AC_SET_MUTE 0x40044103
#define AC_SET_GAIN 0x40044105

static int fd_ai = -1, fd_ao = -1, fd_ac = -1;

int xmv200_audio_open(uint32_t rate, int in, int out) {
  const char *chip = xmv200_chip_name();

  xmv200_audio_close();
  if (in && (fd_ai = open("/dev/ai", O_RDONLY)) < 0)
    return -1;
  if (out && (fd_ao = open("/dev/ao", O_WRONLY)) < 0) {
    xmv200_audio_close();
    return -1;
  }
  fd_ac = open("/dev/acodec", O_RDWR); /* volume and gain, if there */
  if (rate && fd_ai >= 0 && chip &&
      (!strcmp(chip, "xm535d1") || !strcmp(chip, "xm535d2") ||
       !strcmp(chip, "xm535")))
    ioctl(fd_ai, AI_SET_RATE, &rate);
  return 0;
}

void xmv200_audio_close(void) {
  if (fd_ai >= 0)
    close(fd_ai);
  if (fd_ao >= 0)
    close(fd_ao);
  if (fd_ac >= 0)
    close(fd_ac);
  fd_ai = fd_ao = fd_ac = -1;
}

/* i2s.ko hands out at most its FIFO (4096 samples) a read */
int xmv200_audio_read(int16_t *pcm, int samples) {
  uint8_t *p = (uint8_t *)pcm;
  size_t left = samples * 2;

  if (fd_ai < 0)
    return -1;
  while (left) {
    ssize_t n = read(fd_ai, p, left);

    if (n < 0 && errno == EINTR)
      continue;
    if (n <= 0)
      return p == (uint8_t *)pcm ? -1 : (int)((p - (uint8_t *)pcm) / 2);
    p += n;
    left -= n;
  }
  return samples;
}

int xmv200_audio_write(const int16_t *pcm, int samples) {
  const uint8_t *p = (const uint8_t *)pcm;
  size_t left = samples * 2;

  if (fd_ao < 0)
    return -1;
  while (left) {
    ssize_t n = write(fd_ao, p, left);

    if (n < 0 && errno == EINTR)
      continue;
    if (n <= 0)
      return -1;
    p += n;
    left -= n;
  }
  return samples;
}

int xmv200_audio_queued(void) {
  uint32_t n = 0;

  if (fd_ao < 0 || ioctl(fd_ao, AO_GET_USED, &n) < 0)
    return -1;
  return n / 2;
}

static int ac(uint32_t cmd, uint32_t v) {
  return fd_ac < 0 ? -1 : ioctl(fd_ac, cmd, &v);
}

int xmv200_audio_set_volume(int volume) {
  return ac(AC_SET_VOLUME, volume < 0 ? 0 : volume > 100 ? 100 : volume);
}

int xmv200_audio_set_mute(int mute) { return ac(AC_SET_MUTE, !!mute); }

int xmv200_audio_set_gain(int gain) {
  return ac(AC_SET_GAIN, gain < 0 ? 0 : gain > 100 ? 100 : gain);
}

/* ---- G.711 (ITU-T, the reference algorithm) ---- */

uint8_t xmv200_alaw(int16_t pcm) {
  int v = pcm >> 3, mask, seg = 0;

  if (v >= 0) {
    mask = 0xd5;
  } else {
    mask = 0x55;
    v = -v - 1;
  }
  while (seg < 8 && v > (0x20 << seg) - 1)
    seg++;
  if (seg >= 8)
    return 0x7f ^ mask;
  return ((seg << 4) | ((seg ? v >> seg : v >> 1) & 0xf)) ^ mask;
}

int16_t xmv200_alaw_pcm(uint8_t a) {
  int t, seg;

  a ^= 0x55;
  t = (a & 0xf) << 4;
  seg = (a & 0x70) >> 4;
  if (seg == 0)
    t += 8;
  else if (seg == 1)
    t += 0x108;
  else
    t = (t + 0x108) << (seg - 1);
  return a & 0x80 ? t : -t;
}

uint8_t xmv200_ulaw(int16_t pcm) {
  int v = pcm >> 2, mask, seg = 0;

  if (v < 0) {
    v = -v;
    mask = 0x7f;
  } else {
    mask = 0xff;
  }
  if (v > 8159)
    v = 8159;
  v += 33;
  while (seg < 8 && v > (0x40 << seg) - 1)
    seg++;
  if (seg >= 8)
    return 0x7f ^ mask;
  return ((seg << 4) | ((v >> (seg + 1)) & 0xf)) ^ mask;
}

int16_t xmv200_ulaw_pcm(uint8_t u) {
  int t;

  u = ~u;
  t = ((u & 0xf) << 3) + 0x84;
  t <<= (u & 0x70) >> 4;
  return u & 0x80 ? 0x84 - t : t - 0x84;
}

void xmv200_g711_encode(const int16_t *pcm, int n, uint8_t *out, int ulaw) {
  for (int i = 0; i < n; i++)
    out[i] = ulaw ? xmv200_ulaw(pcm[i]) : xmv200_alaw(pcm[i]);
}

void xmv200_g711_decode(const uint8_t *in, int n, int16_t *pcm, int ulaw) {
  for (int i = 0; i < n; i++)
    pcm[i] = ulaw ? xmv200_ulaw_pcm(in[i]) : xmv200_alaw_pcm(in[i]);
}
