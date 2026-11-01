/* SPDX-License-Identifier: MIT */
/*
 * The isptrace.so log: a header, then records, all little-endian. Each
 * record is struct isptrace_rec and len bytes of payload, padded to 4.
 */
#ifndef ISPTRACE_H
#define ISPTRACE_H

#include <stdint.h>

#define ISPTRACE_MAGIC 0x54495358 /* "XSIT" */
#define ISPTRACE_VERSION 2

/* The XM510 /dev/isp commands, unencoded, on {u32 addr; u32 val}. */
#define ISPTRACE_ISP_WRITE 0x101
#define ISPTRACE_ISP_READ 0x103

/* The XM530/XM550 ones, _IOW('I', n, 12) on {u32 addr; u32 val; u32 arg}:
 * a write, two reads (the value written back), and a wait for an event
 * (arg: 0 the Bayer frame, 1 the luma one), whose return is the frame
 * start there: XM530 has no SIGIO. */
#define ISPTRACE_ISP530_WRITE 0x400c4901
#define ISPTRACE_ISP530_READ 0x400c4902
#define ISPTRACE_ISP530_READ2 0x400c4903
#define ISPTRACE_ISP530_WAIT 0x400c4904

/* An ioctl record's snapshots of its argument. */
#define ISPTRACE_BEFORE 1
#define ISPTRACE_AFTER 2

/* A MEMRW record's instruction for a page partly unmapped. */
#define ISPTRACE_PARTIAL_UNMAP 0xffffffffu

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
  ISPTRACE_FRAME = 1, /* aux: frame count; t_us: when the SIGIO came, or
                         (XM530) when the wait for the Bayer frame event
                         returned */
  ISPTRACE_ISPW,      /* aux: register; payload: u32 value */
  ISPTRACE_ISPR,      /* aux: register; payload: u32 value read */
  ISPTRACE_IOCTL,     /* aux: cmd; aux2: result or -errno; payload:
                         u32 arg, u32 flags (ISPTRACE_BEFORE, _AFTER), then
                         the bytes at arg before and after, the same size
                         each, as the flags say they were readable. Version 1
                         had a flag of 1 for both */
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
  ISPTRACE_CALL,   /* a call into the ISP library (XM530 libisp_XM530.so):
                      aux: 0 on the way in, 1 on the way out; aux2: on the
                      way out, the low 16 bits of the result; payload:
                      struct isptrace_call, then ISPTRACE_CALL_SNAP bytes
                      at each argument whose bit is set in mask */
};

#define ISPTRACE_CALL_NAME 32
#define ISPTRACE_CALL_SNAP 512

struct isptrace_call {
  char name[ISPTRACE_CALL_NAME]; /* NUL-padded */
  char from[ISPTRACE_CALL_NAME]; /* the caller's module, its file name */
  uint32_t arg[4];               /* r0-r3 */
  uint32_t mask; /* bit i: arg[i] pointed at readable memory, snapped */
};

#endif
