/* SPDX-License-Identifier: MIT */
/*
 * The isptrace.so log: a header, then records, all little-endian. Each
 * record is struct isptrace_rec and len bytes of payload, padded to 4.
 */
#ifndef ISPTRACE_H
#define ISPTRACE_H

#include <stdint.h>

#define ISPTRACE_MAGIC 0x54495358 /* "XSIT" */
#define ISPTRACE_VERSION 1

/* The XM510 /dev/isp commands, unencoded, on {u32 addr; u32 val}. */
#define ISPTRACE_ISP_WRITE 0x101
#define ISPTRACE_ISP_READ 0x103

/* The most argument bytes an ioctl record keeps, before and after. */
#define ISPTRACE_IOCTL_MAX 256

struct isptrace_hdr {
  uint32_t magic;
  uint32_t version;
  uint32_t t0_us;
  uint32_t reserved;
};

struct isptrace_rec {
  uint32_t t_us;  /* CLOCK_MONOTONIC, microseconds, wrapping */
  uint32_t aux;   /* by type, below */
  uint16_t frame; /* frame starts (SIGIO) seen so far */
  uint8_t type;
  uint8_t fd;
  uint16_t len;  /* payload bytes */
  uint16_t aux2; /* by type: usually the low 16 bits of the result */
};

enum {
  ISPTRACE_FRAME = 1, /* aux: frame count; t_us: when the SIGIO came */
  ISPTRACE_ISPW,      /* aux: register; payload: u32 value */
  ISPTRACE_ISPR,      /* aux: register; payload: u32 value read */
  ISPTRACE_IOCTL,     /* aux: cmd; aux2: result or -errno; payload:
                         u32 arg, u32 readable, then the bytes at arg before
                         and, if readable, after: the same size each */
  ISPTRACE_OPEN,      /* aux: flags; aux2: fd; payload: path */
  ISPTRACE_CREAT,     /* aux: mode; aux2: fd; payload: path */
  ISPTRACE_FOPEN,     /* aux2: opened; payload: path, mode */
  ISPTRACE_ACCESS,    /* aux: mode; aux2: result; payload: path */
  ISPTRACE_CLOSE,
  ISPTRACE_READ,   /* aux: size asked; aux2: result; payload: <= 64 bytes */
  ISPTRACE_MSGSND, /* aux: queue; aux2: result; payload: type, data */
  ISPTRACE_MSGRCV, /* aux: queue; aux2: result; payload: type, data */
  ISPTRACE_XMAP,   /* aux: offset; aux2: 1 if its stores are traced;
                      payload: u32 len, prot, flags, addr */
  ISPTRACE_SIGNAL, /* aux: signal; aux2: 1 if its handler is wrapped,
                      2 when a wrapped one (not SIGIO) is delivered */
  ISPTRACE_MEMW,   /* a store to a /dev/mem register page: aux: physical
                      address; aux2: size; payload: u32 value */
  ISPTRACE_MEMRW,  /* a store it can't decode: aux: the page's physical
                      address; payload: the instruction (0 for Thumb).
                      The page is writable, and untraced, from then on */
};

#endif
