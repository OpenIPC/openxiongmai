/* SPDX-License-Identifier: MIT */
/*
 * The /dev/mv500 interface of the vendor Mali-V550 driver (mve_base.ko)
 * on the XM530/XM550, as far as an H.265 encoder session needs it
 * (OpenIPC/openxiongmai#27).
 *
 * Recovered from the module's command dispatch and from a trace of the
 * vendor library encoding 1920x1080 H.265. The names are ours; only what
 * the trace shows is defined here.
 */
#ifndef MVB_H
#define MVB_H

#include <stdint.h>

/* Every command: struct mvb_req followed by its payload. The reply
 * overwrites the same buffer as struct mvb_reply. */
#define MVB_IOC_CMD 0xC008FB00u
/* 160 bytes of session counters; the vendor library polls it */
#define MVB_IOC_STATS 0xC0A0FB01u

enum mvb_cmd {
  MVB_CREATE_SESSION = 0, /* role string, NUL included */
  MVB_DESTROY_SESSION = 1,
  MVB_ACTIVATE_SESSION = 2, /* reply: u32 firmware protocol, 0x203 = 2.3 */
  MVB_FLUSH_BUFFERS = 3,
  MVB_STATE_CHANGE = 4,  /* u32 state */
  MVB_GET_EVENT = 5,     /* u32 timeout in ms; reply: struct mvb_event */
  MVB_SET_PARAMETER = 6, /* a firmware message, see below */
  MVB_GET_PARAMETER = 7,
  MVB_SET_CONFIG = 8, /* a firmware message, see below */
  MVB_GET_CONFIG = 9,
  MVB_REGISTER_BUFFER = 10,   /* struct mvb_register */
  MVB_UNREGISTER_BUFFER = 11, /* u64 handle */
  MVB_FILL_THIS_BUFFER = 12,  /* struct mvb_queue: an output buffer */
  MVB_EMPTY_THIS_BUFFER = 13, /* struct mvb_queue: an input buffer */
  MVB_NOTIFY_REF_FRAME_RELEASE = 14,
  MVB_REQUEST_MAX_FREQUENCY = 15,
  MVB_READ_HW_INFO = 16, /* reply: u32 fuse, u32 version, u32 cores */
  MVB_RPC_MEM_ALLOC = 17,
  MVB_RPC_MEM_RESIZE = 18,
};

#define MVB_ROLE_HEVC_ENCODER "video_encoder.hevc"

/* MVB_STATE_CHANGE: the encoder runs in state 2 */
#define MVB_STATE_EXECUTING 2

struct mvb_req {
  uint32_t cmd;
  uint32_t size; /* of the payload that follows */
};

struct mvb_reply {
  uint32_t status[2]; /* 0 on success */
  uint32_t size;      /* of the data that follows */
};

/* status[0] of a parameter or config command before the session has
 * first been switched in (MVB_EVENT_SWITCHED_IN) */
#define MVB_STATUS_NOT_SCHEDULED 8

/* Ports of MVB_REGISTER_BUFFER */
#define MVB_PORT_INPUT 0
#define MVB_PORT_OUTPUT 1

/* Buffer formats */
#define MVB_FORMAT_NV12 0x7f000101u /* 4:2:0, Y plane then CbCr */
#define MVB_FORMAT_HEVC 0x7f000107u /* the output bitstream */

/* A buffer, as registered and as the events return it. The handle and
 * address are the buffer's user virtual address; the vendor library
 * passes them sign-extended to 64 bits. Its MMZ is mapped from /dev/mmz,
 * and the driver looks up the physical pages itself. */
struct mvb_buf {
  uint32_t zero0[2];
  uint64_t handle;
  uint64_t addr;
  uint32_t zero1[3];
  uint32_t size;
  uint32_t width, height, stride; /* input only */
  uint32_t planes_or_one;         /* input: 1 */
  uint32_t format;
  uint32_t visible_width, visible_height; /* set by the driver (input) */
  uint32_t zero2[2];
  uint32_t filled;   /* bytes of data, from offset */
  uint32_t offset;   /* of the data in the buffer (output: often ~2 KiB) */
  uint32_t flags;    /* output: MVB_FLAG_* */
  uint32_t in_flags; /* input: what MVB_EMPTY_THIS_BUFFER passed */
  uint32_t zero3[7];
} __attribute__((packed));

/* Output flags seen: 0x480 on the parameter sets, 0x430 on IDR frames,
 * 0x410 on P frames */
#define MVB_FLAG_FRAME_END 0x010u
#define MVB_FLAG_SYNC 0x020u
#define MVB_FLAG_CODEC_CONFIG 0x080u

struct mvb_register {
  uint32_t port;
  struct mvb_buf buf;
} __attribute__((packed));

/* MVB_EMPTY_THIS_BUFFER and MVB_FILL_THIS_BUFFER. The vendor library
 * sends 208 bytes; the rest are zero. Input frames go with filled = the
 * whole picture and in_flags 4; output buffers with filled 0. */
struct mvb_queue {
  uint64_t handle;
  uint64_t addr;
  uint32_t filled;
  uint32_t zero0;
  uint32_t in_flags;
  uint32_t zero1[45];
};
#define MVB_IN_FLAGS 4u

/* MVB_GET_EVENT reply data */
struct mvb_event {
  uint16_t code; /* MVB_EVENT_* */
  uint16_t size; /* of data */
  uint8_t data[];
};

/* Events seen. The session is switched in (1, 4 bytes of zero) and out
 * (2, 12 bytes) around the work of every frame; 4 follows
 * MVB_STATE_CHANGE with the new state; 8 hands an input buffer back and
 * 9 an output buffer filled, both with a struct mvb_buf. */
#define MVB_EVENT_SWITCHED_IN 1
#define MVB_EVENT_SWITCHED_OUT 2
#define MVB_EVENT_STATE_CHANGED 4
#define MVB_EVENT_INPUT_DONE 8
#define MVB_EVENT_OUTPUT_DONE 9

/*
 * MVB_SET_PARAMETER and MVB_SET_CONFIG carry firmware messages that the
 * driver passes through: a u32 message code, then its fields.
 */
#define MVB_FW_SET_OPTION 1010   /* u32 option, u32 value */
#define MVB_FW_BUFFER_PARAM 3003 /* u32 type, then the type's fields */

/* MVB_FW_SET_OPTION, through MVB_SET_PARAMETER. The vendor library sends
 * 0xc0 bytes for this message. */
#define MVB_OPT_P_FRAMES 16 /* P frames after each IDR */
#define MVB_OPT_SIZE 0xc0

/* MVB_FW_BUFFER_PARAM types */
#define MVB_PARAM_FRAME_RATE 9    /* u32 fps << 16; MVB_SET_CONFIG */
#define MVB_PARAM_RATE_CONTROL 10 /* u32 mode, u32 bit/s; MVB_SET_CONFIG */
#define MVB_PARAM_GOP_RESET 18    /* no fields; MVB_SET_CONFIG: next is IDR */
#define MVB_PARAM_QP_RANGE 21     /* s32 min, s32 max; MVB_SET_PARAMETER */

/* MVB_PARAM_RATE_CONTROL modes: the vendor library uses 2 for its
 * variable bitrate mode */
#define MVB_RC_VARIABLE 2

/* The sizes the driver takes */
typedef char mvb_buf_is_120[sizeof(struct mvb_buf) == 120 ? 1 : -1];
typedef char mvb_register_is_124[sizeof(struct mvb_register) == 124 ? 1 : -1];
typedef char mvb_queue_is_208[sizeof(struct mvb_queue) == 208 ? 1 : -1];

#endif
