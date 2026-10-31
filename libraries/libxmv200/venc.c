// SPDX-License-Identifier: MIT
/*
 * H.265 on the V200 series' encoder engines (hevc.ko, /dev/h265, registers
 * at 0x21000000) fed by venc.ko's frame FIFO (/dev/venc_info).
 *
 * Frames: a VPSS channel bound to VENC (module 7) queues each picture in
 * venc.ko; 0xc0284902 takes one (40-byte record, a 2 s wait inside) and
 * 0x40284903 gives it back to its VB pool.
 *
 * Engine: one job encodes one picture as one slice. The job's register list
 * is {value, write} pairs, entry n for the register at 4n; hevc.ko writes
 * them, starts the engine and on its interrupt keeps the slice length for
 * the wait (+44 of the job it returns). 0x4803/0x4804 take and give back
 * the engine. The engine writes only slice_segment_data(); the parameter
 * sets and the slice header are ours.
 *
 * Three engines, with their own register maps (struct engine):
 * - v3, the XM530V200's: CTB 32, SAO; reference and reconstruction as
 *   offsets in a ring from its base;
 * - v1, the XM535D1/D2/W1's: CTB 16, no SAO; absolute addresses, the ring's
 *   bounds in registers, and it scales the source to the picture;
 * - x533, the XM533's: v3's job and list, CTB 16 and no SAO as v1, with
 *   absolute addresses and the ring's bounds in registers of its own.
 * Reference and reconstruction share a ring of one picture plus 96 lines:
 * each picture's reconstruction starts 96 lines below the last one's, so it
 * never overwrites rows the next picture still reads.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"
#include "xmv200_priv.h"

#define VENC_GET_FRAME 0xc0284902
#define VENC_RELEASE_FRAME 0x40284903
#define HEVC_LOCK 0x4803
#define HEVC_UNLOCK 0x4804
#define HEVC_RESET 0x40044806

#define MAX_REGS 350
#define RING_LINES 96
#define HDR_ROOM 0x100 /* the vendor's output starts this far in */

/* venc.ko's frame record */
struct venc_rec {
  uint32_t chn, y, width, height, stride, rsvd;
  uint32_t pts_lo, pts_hi, vb, rsvd2;
};

static const uint16_t v3_tmpl_off[] = {
    0x008, 0x00c, 0x010, 0x014, 0x018, 0x01c, 0x020, 0x024, 0x03c, 0x048,
    0x064, 0x068, 0x06c, 0x070, 0x074, 0x078, 0x07c, 0x080, 0x084, 0x088,
    0x08c, 0x090, 0x094, 0x098, 0x09c, 0x0a0, 0x0a4, 0x0a8, 0x0ac, 0x0b0,
    0x0b4, 0x0b8, 0x0bc, 0x0c0, 0x0c4, 0x0c8, 0x0cc, 0x0d0, 0x0d4, 0x0dc,
    0x0e0, 0x0e4, 0x0e8, 0x0ec, 0x0f0, 0x0f4, 0x0f8, 0x0fc, 0x100, 0x104,
    0x108, 0x10c, 0x110, 0x114, 0x118, 0x11c, 0x120, 0x124, 0x128, 0x12c,
    0x130, 0x134, 0x138, 0x13c, 0x140, 0x144, 0x148, 0x14c, 0x150, 0x154,
    0x158, 0x15c, 0x160, 0x164, 0x168, 0x16c, 0x170, 0x174, 0x178, 0x17c,
    0x180, 0x184, 0x188, 0x18c, 0x190, 0x194, 0x198, 0x19c, 0x1a0, 0x1a4,
    0x1a8, 0x1ac, 0x1b0, 0x1b4, 0x1b8, 0x1bc, 0x1c0, 0x1c4, 0x1c8, 0x1cc,
    0x1d0, 0x288, 0x28c, 0x290, 0x294, 0x298, 0x29c, 0x2a0, 0x2b0, 0x2c4,
    0x2c8, 0x2cc, 0x2dc, 0x2e0, 0x2e4, 0x2e8};
static const uint32_t v3_tmpl_val[] = {
    0x3,        0x3,        0x3,        0x1,        0xe,        0x0,
    0x0,        0x0,        0x0,        0x100000,   0x0,        0x1f,
    0xb,        0x20c4146,  0x728824b,  0xc34e3d0,  0x11fff493, 0x145565da,
    0x1865b71d, 0x1e7e0862, 0x3fffffff, 0x3fffffff, 0x1,        0x28a1,
    0x8000a52,  0x0,        0x0,        0x0,        0x0,        0x0,
    0x0,        0x10,       0x0,        0x0,        0x8,        0x50000,
    0x16aa,     0x10,       0x22,       0x1,        0x0,        0x2,
    0xf0d0c0c,  0xb3,       0x180,      0x169,      0xbb,       0xc0,
    0xc5,       0x16c,      0xcd,       0x27d017,   0x285ffd,   0x2a4fee,
    0x29cfe9,   0x2b3003,   0x221ffd,   0x1e9ffd,   0xa,        0x76,
    0x28a,      0x145,      0xda,       0x66,       0x28f,      0x11c07d,
    0x31f000,   0x28f000,   0x245000,   0x35f000,   0x29a000,   0x20d000,
    0x14,       0x46,       0x15e,      0x1,        0x3,        0x5,
    0xe,        0x28,       0x1,        0x1a,       0x18171717, 0x17161616,
    0x1515,     0x383a4048, 0x4a,       0x1,        0x1,        0x180c,
    0x28282828, 0x20202020, 0x20202020, 0x2020,     0x10101010, 0x1010,
    0x1,        0x0,        0x0,        0x1,        0x0,        0x0,
    0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
    0x1,        0x3e807d0,  0xbb80fa0,  0x138a1770, 0x40008,    0x100020,
    0x400080,   0x820739c};

/* the picture-type presets at 0x2a4..0x2d4: IDR, the first P, later P */
static const uint16_t v3_type_off[] = {0x2a4, 0x2a8, 0x2ac, 0x2b4,
                                    0x2b8, 0x2bc, 0x2d0, 0x2d4};
static const uint32_t v3_idr[] = {0x2f,    0x1c, 0x2b,      0x4,
                                    0x43332, 0x1,  0x8314e95, 0x15};
static const uint32_t v3_p1[] = {0x2,    0x23, 0x26,      0x2d,
                                   0x1ddd, 0xa,  0x8317fbb, 0x1a};
static const uint32_t v3_p[] = {0x3,    0x23, 0x2b,      0x2d,
                                  0x1ddd, 0xa,  0x8317fbb, 0x1a};
/* the same for pictures up to 1280 wide (the vendor's sub stream) */
static const uint32_t v3_idr_s[] = {0x197,   0x1c, 0x2b,      0x4,
                                      0x2cccc, 0x1,  0x8314e95, 0x15};
static const uint32_t v3_p1_s[] = {0x16,   0x23, 0x26,      0xa,
                                     0x13e9, 0xa,  0x8317fbb, 0x1a};
static const uint32_t v3_p_s[] = {0x1c,   0x23, 0x2b,      0xa,
                                    0x13e9, 0xa,  0x8317fbb, 0x1a};

/* v1 (XM535D1/D2): from the vendor encoder's jobs */
static const uint16_t v1_tmpl_off[] = {
    0x000, 0x018, 0x01c, 0x020, 0x024, 0x028, 0x02c, 0x030, 0x034, 0x038,
    0x03c, 0x040, 0x04c, 0x050, 0x080, 0x084, 0x08c, 0x090, 0x094, 0x098,
    0x09c, 0x108, 0x10c, 0x110, 0x114, 0x118, 0x11c, 0x220, 0x32c, 0x330,
    0x338, 0x33c, 0x348, 0x34c, 0x350, 0x36c, 0x370, 0x374, 0x378, 0x37c,
    0x380, 0x384, 0x398, 0x39c, 0x3a0, 0x3a4, 0x3a8, 0x3ac, 0x3e0, 0x3fc,
    0x400, 0x404, 0x420, 0x424, 0x428, 0x42c, 0x430, 0x434, 0x438, 0x43c,
    0x440, 0x444, 0x448, 0x44c, 0x450, 0x454, 0x458, 0x45c, 0x460, 0x464,
    0x468, 0x46c, 0x470, 0x474, 0x478, 0x47c, 0x480, 0x484, 0x488, 0x48c,
    0x490, 0x498, 0x49c, 0x4a0, 0x4a4, 0x4a8, 0x4ac, 0x4b0, 0x4b4, 0x4b8,
    0x4bc, 0x4c0, 0x4c4, 0x4c8, 0x4cc, 0x4d0, 0x4d4, 0x4d8, 0x4dc, 0x4e0,
    0x4e4, 0x4e8, 0x4ec, 0x4f0, 0x4f4, 0x4f8, 0x4fc, 0x500, 0x504, 0x508,
    0x50c, 0x510, 0x514, 0x518, 0x51c, 0x520, 0x524, 0x528, 0x534};
static const uint32_t v1_tmpl_val[] = {
    0x0,        0x3,        0x3,        0x3,        0x0,        0x3,
    0x7,        0x0,        0x0,        0x0,        0x0,        0x271000,
    0xff,       0xff,       0x1,        0x0,        0xffffffff, 0x0,
    0x0,        0x1,        0x0,        0x0,        0xd555,     0xd555,
    0xc8,       0x148,      0x0,        0x0,        0x3,        0x1,
    0x40,       0x30,       0x3e807d0,  0xbb80fa0,  0x138a1770, 0x1,
    0x40008,    0xc0010,    0x14001a,   0x200028,   0x30003c,   0x480054,
    0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
    0x1,        0x1,        0x1,        0x180c,     0xf,        0x10,
    0x210620a,  0xc390494,  0x1661a71e, 0x208bffff, 0x3fffffff, 0x3fffffff,
    0x3fffffff, 0x3fffffff, 0x3fffffff, 0x0,        0x20,       0x1450,
    0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
    0x0,        0x0,        0x0,        0x0,        0x22,       0x124924,
    0x2aaa,     0x1,        0x20,       0x1,        0x0,        0x2,
    0xf0d0c0c,  0xb3,       0x180,      0x169,      0xbb,       0xc0,
    0xc5,       0x16c,      0xcd,       0x27d017,   0x285ffd,   0x2a4fee,
    0x29cfe9,   0x2b3003,   0x221ffd,   0x1e9ffd,   0xa,        0x76,
    0x28a,      0x145,      0xda,       0x66,       0x28f,      0x11c07d,
    0x31f000,   0x28f000,   0x245000,   0x35f000,   0x29a000,   0x20d000,
    0x14,       0x14,       0x5,        0xe,        0x0};
static const uint16_t v1_type_off[] = {0x324, 0x328, 0x334, 0x340, 0x344,
                                       0x360, 0x364, 0x388, 0x38c, 0x390};
static const uint32_t v1_idr[] = {0x28,      0x2e, 0x436,     0xa,
                                  0x1,       0x83103fe, 0x1e, 0xe50739c,
                                  0x3bdef7bd, 0x3bdef7bd};
static const uint32_t v1_p1[] = {0x23,      0x26, 0x3b,      0x8,
                                 0x1,       0x83103fe, 0x1e, 0xe50739c,
                                 0x3bdef7bd, 0x3bdef7bd};
static const uint32_t v1_p[] = {0x23,      0x2b,       0x3b,     0x8,
                                0x0,       0x316a08c8, 0x8,      0x8310411,
                                0x253a56d7, 0x319d6f9d};
static const uint32_t v1_idr_s[] = {0x28,      0x2e, 0x2468,    0xa,
                                    0x1,       0x83103fe, 0x1e, 0xe50739c,
                                    0x3bdef7bd, 0x3bdef7bd};
static const uint32_t v1_p1_s[] = {0x23,      0x26, 0x205,     0x8,
                                   0x1,       0x83103fe, 0x1e, 0xe50739c,
                                   0x3bdef7bd, 0x3bdef7bd};
static const uint32_t v1_p_s[] = {0x23,      0x2b,       0x205,    0x8,
                                  0x0,       0x316a08c8, 0x8,      0x8310411,
                                  0x253a56d7, 0x319d6f9d};

/* x533 (XM533): from the vendor encoder's jobs; its presets at the v3 ones'
 * offsets plus 0x2c0 */
static const uint16_t x533_tmpl_off[] = {
    0x008, 0x00c, 0x010, 0x014, 0x018, 0x01c, 0x020, 0x024, 0x03c, 0x048, 0x064,
    0x068, 0x06c, 0x070, 0x074, 0x078, 0x07c, 0x080, 0x084, 0x088, 0x08c, 0x090,
    0x094, 0x098, 0x09c, 0x0a0, 0x0a4, 0x0a8, 0x0ac, 0x0b0, 0x0b4, 0x0b8, 0x0bc,
    0x0c0, 0x0c4, 0x0c8, 0x0cc, 0x0d0, 0x0d4, 0x0dc, 0x0e0, 0x0e4, 0x0e8, 0x0ec,
    0x0f0, 0x0f4, 0x0f8, 0x0fc, 0x100, 0x104, 0x108, 0x10c, 0x110, 0x114, 0x118,
    0x11c, 0x120, 0x124, 0x128, 0x12c, 0x130, 0x134, 0x138, 0x13c, 0x140, 0x144,
    0x148, 0x14c, 0x150, 0x154, 0x158, 0x15c, 0x160, 0x164, 0x16c, 0x174, 0x178,
    0x17c, 0x180, 0x184, 0x188, 0x18c, 0x190, 0x194, 0x198, 0x19c, 0x1a0, 0x1a4,
    0x1a8, 0x1ac, 0x1b0, 0x1b4, 0x1b8, 0x1bc, 0x1c0, 0x1c8, 0x1cc, 0x1d0, 0x288,
    0x28c, 0x290, 0x294, 0x298, 0x29c, 0x2a0, 0x2b0, 0x2c4, 0x2c8, 0x2cc, 0x2d4,
    0x2d8, 0x2dc, 0x2e0, 0x2e4, 0x2e8, 0x2ec};
static const uint32_t x533_tmpl_val[] = {
    0x3, 0x3, 0x3, 0x1, 0x7, 0x0, 0x0, 0x0, 0x0, 0x100000, 0x0, 0xf, 0x20,
    0x20c4146, 0x720928b, 0xc34e3d0, 0x11493515, 0x165d865a, 0x1b71d79f,
    0x20862fff, 0x3fffffff, 0x3fffffff, 0x0, 0xe66, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x10, 0x0, 0x0, 0xc, 0x53924, 0x14aa, 0x10, 0x22, 0x1, 0x0, 0x2,
    0xf0d0c0c, 0xb3, 0x180, 0x169, 0xbb, 0xc0, 0xc5, 0x16c, 0xcd, 0x27d017,
    0x285ffd, 0x2a4fee, 0x29cfe9, 0x2b3003, 0x221ffd, 0x1e9ffd, 0xa, 0x76,
    0x28a, 0x145, 0xda, 0x66, 0x28f, 0x11c07d, 0x31f000, 0x28f000, 0x245000,
    0x35f000, 0x29a000, 0x20d000, 0x14, 0x46, 0x1, 0x5, 0xe, 0x28, 0x1, 0x18,
    0x17161616, 0x15151515, 0x1413, 0x383a4048, 0x4a, 0x1, 0x1, 0x180c,
    0x28282828, 0x20202020, 0x20202020, 0x2020, 0x10101819, 0x1d1c, 0x1, 0x0,
    0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x3e807d0, 0xbb80fa0,
    0x138a1770, 0x1e, 0x0, 0x40008, 0x100020, 0x400080, 0x820739c, 0x1d};
static const uint16_t x533_type_off[] = {
    0x2a4, 0x2a8, 0x2ac, 0x2b4, 0x2b8, 0x2bc, 0x2c0, 0x2d0};
static const uint32_t x533_idr[] = {
    0x4b7, 0x1c, 0x29, 0x4, 0x3e8, 0x1, 0x1, 0x83103fe};
static const uint32_t x533_p1[] = {
    0x92, 0x1a, 0x26, 0x16, 0x2c8, 0xa, 0x1, 0x83103fe};
static const uint32_t x533_p[] = {
    0xe0, 0x1a, 0x26, 0x16, 0x2c8, 0xa, 0x0, 0x8310000};
static const uint32_t x533_idr_s[] = {
    0x1892, 0x1c, 0x29, 0x4, 0x3e8, 0x1, 0x0, 0x83103fe};
static const uint32_t x533_p1_s[] = {
    0x28f, 0x1a, 0x26, 0xa, 0x2c8, 0xa, 0x0, 0x83103fe};
static const uint32_t x533_p_s[] = {
    0x3a6, 0x1a, 0x26, 0xa, 0x2c8, 0xa, 0x0, 0x8310000};

/* v1: the rate-distortion tables the vendor encoder writes from user space
 * (+0x58 selects a bank, +0x800.. its entries), the same for every picture */
static const struct {
  uint16_t off;
  uint32_t val;
} v1_rd[] = {
    {0x058, 0x0}, {0x800, 0x6704}, {0x804, 0x627e}, {0x808, 0x5da0},
    {0x80c, 0x596b}, {0x810, 0x5409}, {0x814, 0x4edf}, {0x818, 0x4a48},
    {0x81c, 0x4397}, {0x820, 0x3e59}, {0x824, 0x39d0}, {0x828, 0x342f},
    {0x82c, 0x3074}, {0x830, 0x2c5c}, {0x834, 0x2899}, {0x838, 0x24b9},
    {0x83c, 0x21c6}, {0x840, 0x1ede}, {0x844, 0x1c66}, {0x848, 0x19ce},
    {0x84c, 0x16d8}, {0x850, 0x12b1}, {0x854, 0xf69}, {0x858, 0xc62},
    {0x85c, 0xa50}, {0x860, 0x878}, {0x864, 0x712}, {0x868, 0x5e0},
    {0x86c, 0x50b}, {0x870, 0x44f}, {0x874, 0x3c0}, {0x878, 0x356},
    {0x87c, 0x2f4}, {0x880, 0x297}, {0x884, 0x24d}, {0x888, 0x20a},
    {0x88c, 0x1d8}, {0x890, 0x1a7}, {0x894, 0x17e}, {0x898, 0x152},
    {0x89c, 0x132}, {0x8a0, 0x112}, {0x8a4, 0xf8}, {0x8a8, 0xdd},
    {0x8ac, 0xc9}, {0x8b0, 0xb1}, {0x8b4, 0x9e}, {0x8b8, 0x8b},
    {0x8bc, 0x7b}, {0x8c0, 0x70}, {0x8c4, 0x62}, {0x8c8, 0x57},
    {0x8cc, 0x4f}, {0x8d0, 0x6723}, {0x8d4, 0x629a}, {0x8d8, 0x5dd9},
    {0x8dc, 0x599d}, {0x8e0, 0x54bc}, {0x8e4, 0x4fb5}, {0x8e8, 0x4af0},
    {0x8ec, 0x444e}, {0x8f0, 0x3ed0}, {0x8f4, 0x3a14}, {0x8f8, 0x33fc},
    {0x8fc, 0x3039}, {0x900, 0x2bf1}, {0x904, 0x2804}, {0x908, 0x2413},
    {0x90c, 0x2111}, {0x910, 0x1e04}, {0x914, 0x1b76}, {0x918, 0x1746},
    {0x91c, 0x1264}, {0x920, 0xd12}, {0x924, 0x920}, {0x928, 0x68a},
    {0x92c, 0x4ff}, {0x930, 0x3f4}, {0x934, 0x322}, {0x938, 0x282},
    {0x93c, 0x20a}, {0x940, 0x1a9}, {0x944, 0x163}, {0x948, 0x135},
    {0x94c, 0x109}, {0x950, 0xe5}, {0x954, 0xc5}, {0x958, 0xaa},
    {0x95c, 0x96}, {0x960, 0x82}, {0x964, 0x73}, {0x968, 0x64},
    {0x96c, 0x59}, {0x970, 0x50}, {0x974, 0x47}, {0x978, 0x3f},
    {0x97c, 0x3a}, {0x980, 0x33}, {0x984, 0x2e}, {0x988, 0x29},
    {0x98c, 0x25}, {0x990, 0x22}, {0x994, 0x20}, {0x998, 0x1d},
    {0x99c, 0x1b}, {0x058, 0x1}, {0x800, 0x9}, {0x804, 0xc},
    {0x808, 0xf}, {0x80c, 0x13}, {0x810, 0x18}, {0x814, 0x1f},
    {0x818, 0x27}, {0x81c, 0x32}, {0x820, 0x3f}, {0x824, 0x4f},
    {0x828, 0x64}, {0x82c, 0x7f}, {0x830, 0xa1}, {0x834, 0xcb},
    {0x838, 0x101}, {0x83c, 0x145}, {0x840, 0x19b}, {0x844, 0x207},
    {0x848, 0x290}, {0x84c, 0x33d}, {0x850, 0x418}, {0x854, 0x52c},
    {0x858, 0x68a}, {0x85c, 0x843}, {0x860, 0xa71}, {0x864, 0xd31},
    {0x868, 0x10ac}, {0x86c, 0x1511}, {0x870, 0x1a9f}, {0x874, 0x21a4},
    {0x878, 0x2a83}, {0x87c, 0x35b8}, {0x880, 0x43e2}, {0x884, 0x55c8},
    {0x888, 0x6c66}, {0x88c, 0x88fa}, {0x890, 0xad17}, {0x894, 0xdaba},
    {0x898, 0x11465}, {0x89c, 0x15d43}, {0x8a0, 0x1b958}, {0x8a4, 0x22db4},
    {0x8a8, 0x2c0bd}, {0x8ac, 0x37a8a}, {0x8b0, 0x46554}, {0x8b4, 0x58e04},
    {0x8b8, 0x704ec}, {0x8bc, 0x8deab}, {0x8c0, 0xb3552}, {0x8c4, 0xe29cd},
    {0x8c8, 0x11e5b9}, {0x8cc, 0x169dab}, {0x058, 0x2}, {0x800, 0x20},
    {0x804, 0x20}, {0x808, 0x20}, {0x80c, 0x20}, {0x810, 0x20},
    {0x814, 0x20}, {0x818, 0x20}, {0x81c, 0x20}, {0x820, 0x20},
    {0x824, 0x20}, {0x828, 0x20}, {0x82c, 0x20}, {0x830, 0x20},
    {0x834, 0x20}, {0x838, 0x20}, {0x83c, 0x20}, {0x840, 0x20},
    {0x844, 0x20}, {0x848, 0x20}, {0x84c, 0x20}, {0x850, 0x20},
    {0x854, 0x20}, {0x858, 0x20}, {0x85c, 0x20}, {0x860, 0x20},
    {0x864, 0x20}, {0x868, 0x20}, {0x86c, 0x20}, {0x870, 0x20},
    {0x874, 0x20}, {0x878, 0x20}, {0x87c, 0x20}, {0x880, 0x20},
    {0x884, 0x20}, {0x888, 0x20}, {0x88c, 0x20}, {0x890, 0x20},
    {0x894, 0x20}, {0x898, 0x20}, {0x89c, 0x20}, {0x8a0, 0x20},
    {0x8a4, 0x20}, {0x8a8, 0x20}, {0x8ac, 0x20}, {0x8b0, 0x20},
    {0x8b4, 0x20}, {0x8b8, 0x20}, {0x8bc, 0x20}, {0x8c0, 0x20},
    {0x8c4, 0x20}, {0x8c8, 0x20}, {0x8cc, 0x20}, {0x058, 0x3},
    {0x800, 0x20}, {0x804, 0x20}, {0x808, 0x20}, {0x80c, 0x20},
    {0x810, 0x20}, {0x814, 0x20}, {0x818, 0x20}, {0x81c, 0x20},
    {0x820, 0x20}, {0x824, 0x20}, {0x828, 0x20}, {0x82c, 0x20},
    {0x830, 0x20}, {0x834, 0x20}, {0x838, 0x20}, {0x83c, 0x20},
    {0x840, 0x20}, {0x844, 0x20}, {0x848, 0x20}, {0x84c, 0x20},
    {0x850, 0x20}, {0x854, 0x20}, {0x858, 0x20}, {0x85c, 0x20},
    {0x860, 0x20}, {0x864, 0x20}, {0x868, 0x20}, {0x86c, 0x20},
    {0x870, 0x20}, {0x874, 0x20}, {0x878, 0x20}, {0x87c, 0x20},
    {0x880, 0x20}, {0x884, 0x20}, {0x888, 0x20}, {0x88c, 0x20},
    {0x890, 0x20}, {0x894, 0x20}, {0x898, 0x20}, {0x89c, 0x20},
    {0x8a0, 0x20}, {0x8a4, 0x20}, {0x8a8, 0x20}, {0x8ac, 0x20},
    {0x8b0, 0x20}, {0x8b4, 0x20}, {0x8b8, 0x20}, {0x8bc, 0x20},
    {0x8c0, 0x20}, {0x8c4, 0x20}, {0x8c8, 0x20}, {0x8cc, 0x20},
    {0x8d0, 0x20}, {0x8d4, 0x20}, {0x8d8, 0x20}, {0x8dc, 0x20},
    {0x8e0, 0x20}, {0x8e4, 0x20}, {0x8e8, 0x20}, {0x8ec, 0x20},
    {0x8f0, 0x20}, {0x8f4, 0x20}, {0x8f8, 0x20}, {0x8fc, 0x20},
    {0x058, 0x4}, {0x800, 0x20}, {0x804, 0x20}, {0x808, 0x20},
    {0x80c, 0x20}, {0x810, 0x20}, {0x814, 0x20}, {0x818, 0x20},
    {0x81c, 0x20}, {0x820, 0x20}, {0x824, 0x20}, {0x828, 0x20},
    {0x82c, 0x20}, {0x830, 0x20}, {0x834, 0x20}, {0x838, 0x20},
    {0x83c, 0x20}, {0x840, 0x20}, {0x844, 0x20}, {0x848, 0x20},
    {0x84c, 0x20}, {0x850, 0x20}, {0x854, 0x20}, {0x858, 0x20},
    {0x85c, 0x20}, {0x860, 0x20}, {0x864, 0x20}, {0x868, 0x20},
    {0x86c, 0x20}, {0x870, 0x20}, {0x874, 0x20}, {0x878, 0x20},
    {0x87c, 0x20}, {0x880, 0x20}, {0x884, 0x20}, {0x888, 0x20},
    {0x88c, 0x20}, {0x890, 0x20}, {0x894, 0x20}, {0x898, 0x20},
    {0x89c, 0x20}, {0x8a0, 0x20}, {0x8a4, 0x20}, {0x8a8, 0x20},
    {0x8ac, 0x20}, {0x8b0, 0x20}, {0x8b4, 0x20}, {0x8b8, 0x20},
    {0x8bc, 0x20}, {0x8c0, 0x20}, {0x8c4, 0x20}, {0x8c8, 0x20},
    {0x8cc, 0x20}, {0x058, 0x5}, {0x800, 0x20}, {0x804, 0x20},
    {0x808, 0x20}, {0x80c, 0x20}, {0x810, 0x20}, {0x814, 0x20},
    {0x818, 0x20}, {0x81c, 0x20}, {0x820, 0x20}, {0x824, 0x20},
    {0x828, 0x20}, {0x82c, 0x20}, {0x830, 0x20}, {0x834, 0x20},
    {0x838, 0x20}, {0x83c, 0x20}, {0x840, 0x20}, {0x844, 0x20},
    {0x848, 0x20}, {0x84c, 0x20}, {0x850, 0x20}, {0x854, 0x20},
    {0x858, 0x20}, {0x85c, 0x20}, {0x860, 0x20}, {0x864, 0x20},
    {0x868, 0x20}, {0x86c, 0x20}, {0x870, 0x20}, {0x874, 0x20},
    {0x878, 0x20}, {0x87c, 0x20}, {0x880, 0x20}, {0x884, 0x20},
    {0x888, 0x20}, {0x88c, 0x20}, {0x890, 0x20}, {0x894, 0x20},
    {0x898, 0x20}, {0x89c, 0x20}, {0x8a0, 0x20}, {0x8a4, 0x20},
    {0x8a8, 0x20}, {0x8ac, 0x20}, {0x8b0, 0x20}, {0x8b4, 0x20},
    {0x8b8, 0x20}, {0x8bc, 0x20}, {0x8c0, 0x20}, {0x8c4, 0x20},
    {0x8c8, 0x20}, {0x8cc, 0x20}, {0x8d0, 0x20}, {0x8d4, 0x20},
    {0x8d8, 0x20}, {0x8dc, 0x20}, {0x8e0, 0x20}, {0x8e4, 0x20},
    {0x8e8, 0x20}, {0x8ec, 0x20}, {0x8f0, 0x20}, {0x8f4, 0x20},
    {0x8f8, 0x20}, {0x8fc, 0x20},
};

struct engine {
  const char *name;
  uint32_t job_cmd, wait_cmd, job_size; /* hevc.ko's job */
  int list_word;                        /* the list pointer in the job */
  int nregs;                            /* entries in the list */
  int ctb, sao;
  enum { MAP_V3, MAP_V1, MAP_X533 } map; /* the register map */
  int qp_i_up;  /* the IDR's QP above the P pictures' */
  /* the template, and the IDR / first P / later P presets (and theirs for
   * pictures up to 1280 wide) */
  const uint16_t *tmpl_off;
  const uint32_t *tmpl_val;
  int n_tmpl;
  const uint16_t *type_off;
  int n_type;
  const uint32_t *preset[2][3];
};

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
static const struct engine engines[] = {
    {"v3", 0x40384805, 0xc0344802, 0x38, 13, 189, 32, 1, MAP_V3, 0, v3_tmpl_off,
     v3_tmpl_val, N(v3_tmpl_off), v3_type_off, N(v3_type_off),
     {{v3_idr, v3_p1, v3_p}, {v3_idr_s, v3_p1_s, v3_p_s}}},
    {"v1", 0x40484805, 0xc0444802, 0x48, 17, 350, 16, 0, MAP_V1, 3, v1_tmpl_off,
     v1_tmpl_val, N(v1_tmpl_off), v1_type_off, N(v1_type_off),
     {{v1_idr, v1_p1, v1_p}, {v1_idr_s, v1_p1_s, v1_p_s}}},
    {"x533", 0x40384805, 0xc0344802, 0x38, 13, 189, 16, 0, MAP_X533, 3,
     x533_tmpl_off, x533_tmpl_val, N(x533_tmpl_off), x533_type_off,
     N(x533_type_off),
     {{x533_idr, x533_p1, x533_p}, {x533_idr_s, x533_p1_s, x533_p_s}}},
};

static const struct engine *the_engine;

/* the H1's H.264 (h264.c): an engine only for rate control's sake */
static const struct engine h1_engine = {"h1"};

/* the chip's engine: the board's, or by the kernel's machine name */
void xmv200_venc_set_engine(const char *chip) {
  the_engine = !strncmp(chip, "xm535", 5)   ? &engines[1]
               : !strncmp(chip, "xm533", 5) ? &engines[2]
                                            : &engines[0];
}

static const struct engine *engine(void) {
  if (!the_engine) {
    char line[128];
    FILE *f = fopen("/proc/cpuinfo", "r");

    xmv200_venc_set_engine("xm530v200");
    while (f && fgets(line, sizeof(line), f))
      if (!strncmp(line, "Hardware", 8) && strstr(line, "xm535"))
        xmv200_venc_set_engine("xm535");
      else if (!strncmp(line, "Hardware", 8) && strstr(line, "xm533"))
        xmv200_venc_set_engine("xm533");
    if (f)
      fclose(f);
  }
  return the_engine;
}

struct xmv200_venc {
  const struct engine *e;
  struct h264enc *h264; /* H.264 on the H1, not H.265 */
  int fd_hevc, fd_mem;
  uint32_t width, height, hc, qp, qp_i, gop, fps;
  struct xmv200_buf luma, chroma; /* reference/reconstruction rings */
  struct xmv200_buf stream;
  uint8_t *stream_map;
  uint32_t frame, recon, poc;
  uint32_t regs[MAX_REGS][2];
  /* rate control */
  struct xmv200_venc_rc rc;
  double fill;           /* bits over the budget so far, a second at most */
  double avg_p, i_over_p; /* a P picture's bits on average; an IDR's over it */
  uint32_t win_bytes[64]; /* the last second's pictures */
  uint64_t frames, bytes;
};

/* ---- bitstream ---- */

struct bw {
  uint8_t *p;
  int cap, len; /* bytes */
  uint64_t acc;
  int nacc;
};

static void put(struct bw *b, uint32_t v, int n) {
  while (n--) {
    b->acc = (b->acc << 1) | ((v >> n) & 1);
    if (++b->nacc == 8) {
      if (b->len < b->cap)
        b->p[b->len] = (uint8_t)b->acc;
      b->len++;
      b->nacc = 0;
      b->acc = 0;
    }
  }
}

static void ue(struct bw *b, uint32_t v) {
  int n = 0;

  for (uint32_t t = v + 1; t > 1; t >>= 1)
    n++;
  put(b, 0, n);
  put(b, v + 1, n + 1);
}

static void se(struct bw *b, int v) { ue(b, v > 0 ? 2 * v - 1 : -2 * v); }

static void trailing(struct bw *b) {
  put(b, 1, 1);
  while (b->nacc)
    put(b, 0, 1);
}

/* start code + NAL header, then the RBSP with emulation prevention */
static int nal(uint8_t *out, int cap, int type, const uint8_t *rbsp, int len) {
  int n = 0, zeros = 0;

  if (cap < 6 + len * 3 / 2)
    return -1;
  out[n++] = 0, out[n++] = 0, out[n++] = 0, out[n++] = 1;
  out[n++] = type << 1;
  out[n++] = 1;
  for (int i = 0; i < len; i++) {
    if (zeros >= 2 && rbsp[i] <= 3) {
      out[n++] = 3;
      zeros = 0;
    }
    out[n++] = rbsp[i];
    zeros = rbsp[i] ? 0 : zeros + 1;
  }
  return n;
}

static void ptl(struct bw *b) {
  put(b, 0, 2);       /* profile_space */
  put(b, 0, 1);       /* tier */
  put(b, 1, 5);       /* Main */
  put(b, 0x60000000, 32); /* compatible with Main and Main 10 */
  put(b, 0, 4);       /* progressive/interlaced/non-packed/frame-only */
  put(b, 0, 32);
  put(b, 0, 12);
  put(b, 63, 8); /* level 2.1 as the vendor encoder writes it */
}

static int write_vps(struct xmv200_venc *v, uint8_t *rbsp, int cap) {
  struct bw b = {rbsp, cap};

  put(&b, 0, 4);      /* vps_id */
  put(&b, 3, 2);      /* base layer internal + available */
  put(&b, 0, 6);      /* max_layers_minus1 */
  put(&b, 0, 3);      /* max_sub_layers_minus1 */
  put(&b, 1, 1);      /* temporal_id_nesting */
  put(&b, 0xffff, 16);
  ptl(&b);
  put(&b, 0, 1);      /* sub_layer_ordering_info_present */
  ue(&b, 1);          /* max_dec_pic_buffering_minus1 */
  ue(&b, 0);
  ue(&b, 0);
  put(&b, 0, 6);      /* max_layer_id */
  ue(&b, 0);          /* num_layer_sets_minus1 */
  put(&b, 1, 1);      /* timing_info_present */
  put(&b, 1000, 32);
  put(&b, v->fps * 1000, 32);
  put(&b, 0, 1);
  ue(&b, 0);          /* num_hrd_parameters */
  put(&b, 0, 1);      /* extension */
  trailing(&b);
  return b.len;
}

static int write_sps(struct xmv200_venc *v, uint8_t *rbsp, int cap) {
  struct bw b = {rbsp, cap};

  put(&b, 0, 4);      /* vps_id */
  put(&b, 0, 3);      /* max_sub_layers_minus1 */
  put(&b, 1, 1);
  ptl(&b);
  ue(&b, 0);          /* sps_id */
  ue(&b, 1);          /* 4:2:0 */
  ue(&b, v->width);
  ue(&b, v->height);
  put(&b, 1, 1);      /* conformance window, all zero as the vendor's */
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);          /* bit depths */
  ue(&b, 0);
  ue(&b, 6);          /* log2_max_poc_lsb_minus4: 10 bits */
  put(&b, 0, 1);
  ue(&b, 1);
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);          /* min CB 8 */
  ue(&b, v->e->ctb == 32 ? 2 : 1); /* CTB */
  ue(&b, 0);          /* min TB 4 */
  ue(&b, v->e->ctb == 32 ? 3 : 2); /* max TB = CTB */
  ue(&b, 0);          /* transform hierarchy depths */
  ue(&b, 0);
  put(&b, 0, 1);      /* scaling lists */
  put(&b, 0, 1);      /* AMP */
  put(&b, v->e->sao, 1); /* SAO */
  put(&b, 0, 1);      /* PCM */
  ue(&b, 1);          /* one short-term RPS: the previous picture */
  ue(&b, 1);
  ue(&b, 0);
  ue(&b, 0);
  put(&b, 1, 1);
  put(&b, 0, 1);      /* long-term refs */
  put(&b, 0, 1);      /* TMVP */
  put(&b, 0, 1);      /* strong intra smoothing */
  put(&b, 1, 1);      /* VUI: timing only */
  put(&b, 0, 8);
  put(&b, 1, 1);
  put(&b, 1000, 32);
  put(&b, v->fps * 1000, 32);
  put(&b, 0, 1);
  put(&b, 0, 1);      /* HRD */
  put(&b, 0, 1);      /* bitstream restriction */
  put(&b, 0, 1);      /* extension */
  trailing(&b);
  return b.len;
}

static int write_pps(uint8_t *rbsp, int cap) {
  struct bw b = {rbsp, cap};

  ue(&b, 0);
  ue(&b, 0);
  put(&b, 0, 1);      /* dependent slices */
  put(&b, 0, 1);      /* output flag */
  put(&b, 0, 3);
  put(&b, 0, 1);      /* sign data hiding */
  put(&b, 1, 1);      /* cabac_init_present */
  ue(&b, 0);
  ue(&b, 0);
  se(&b, 0);          /* init_qp 26 */
  put(&b, 0, 1);
  put(&b, 0, 1);      /* transform skip */
  put(&b, 1, 1);      /* cu_qp_delta */
  ue(&b, 0);
  se(&b, 0);
  se(&b, 0);
  put(&b, 0, 1);
  put(&b, 0, 1);      /* weighted pred */
  put(&b, 0, 1);
  put(&b, 0, 1);      /* transquant bypass */
  put(&b, 0, 1);      /* tiles */
  put(&b, 0, 1);      /* WPP */
  put(&b, 1, 1);      /* loop filter across slices */
  put(&b, 1, 1);      /* deblocking control */
  put(&b, 1, 1);      /* override enabled */
  put(&b, 0, 1);
  se(&b, 5);          /* beta */
  se(&b, -2);         /* tc */
  put(&b, 0, 1);
  put(&b, 0, 1);
  ue(&b, 0);
  put(&b, 0, 1);
  put(&b, 0, 1);
  trailing(&b);
  return b.len;
}

static int write_slice_header(struct xmv200_venc *v, int idr, uint8_t *rbsp,
                              int cap) {
  struct bw b = {rbsp, cap};

  put(&b, 1, 1);      /* first slice */
  if (idr)
    put(&b, 0, 1);    /* no_output_of_prior_pics */
  ue(&b, 0);          /* pps */
  ue(&b, idr ? 2 : 1);
  if (!idr) {
    put(&b, v->poc & 0x3ff, 10);
    put(&b, 1, 1);    /* the SPS's RPS */
  }
  if (v->e->sao) {
    put(&b, 1, 1);    /* SAO luma, chroma */
    put(&b, 1, 1);
  }
  if (!idr) {
    put(&b, 0, 1);    /* num_ref_idx override */
    put(&b, 0, 1);    /* cabac_init_flag */
    ue(&b, 1);        /* five_minus_max_num_merge_cand */
  }
  se(&b, (int)(idr ? v->qp_i : v->qp) - 26);
  put(&b, 1, 1);      /* deblocking override */
  put(&b, 0, 1);
  se(&b, 5);
  se(&b, -2);
  put(&b, 1, 1);      /* loop filter across slices */
  trailing(&b);       /* byte_alignment() */
  return b.len;
}

/* ---- engine ---- */


/* ---- engine ---- */

static void reg(struct xmv200_venc *v, uint32_t off, uint32_t val) {
  v->regs[off / 4][0] = val;
  v->regs[off / 4][1] = 1;
}

static uint32_t lsize(const struct xmv200_venc *v) { return v->width * v->hc; }
static uint32_t ring(const struct xmv200_venc *v) {
  return lsize(v) + v->width * RING_LINES;
}

static void venc_regs(struct xmv200_venc *v) {
  const struct engine *e = v->e;
  uint32_t lsz = lsize(v), rsz = ring(v);

  memset(v->regs, 0, sizeof(v->regs));
  for (int i = 0; i < e->n_tmpl; i++)
    reg(v, e->tmpl_off[i], e->tmpl_val[i]);
  if (e->map == MAP_V3) {
    reg(v, 0x034, v->width - 1);
    reg(v, 0x038, v->height - 1);
    reg(v, 0x044, v->qp);
    reg(v, 0x1e8, v->luma.phys); /* reference ring */
    reg(v, 0x1ec, v->chroma.phys);
    reg(v, 0x1f0, v->luma.phys); /* reconstruction ring (the same) */
    reg(v, 0x1f4, v->chroma.phys);
    reg(v, 0x230, lsz);
    reg(v, 0x234, lsz / 2);
    reg(v, 0x238, rsz);
    reg(v, 0x23c, rsz / 2);
    for (uint32_t o = 0x214; o <= 0x228; o += 4)
      reg(v, o, v->width);
    reg(v, 0x2c0, v->width > 1280);
    reg(v, 0x2d8, v->width > 1280);
  } else if (e->map == MAP_X533) {
    reg(v, 0x034, v->width - 1);
    reg(v, 0x038, v->height - 1);
    reg(v, 0x200, v->luma.phys + rsz); /* the rings' ends and starts */
    reg(v, 0x204, v->luma.phys);
    reg(v, 0x208, v->chroma.phys + rsz / 2);
    reg(v, 0x20c, v->chroma.phys);
    for (uint32_t o = 0x22c; o <= 0x238; o += 4)
      reg(v, o, v->width * 4);
  } else {
    reg(v, 0x044, v->width - 1);
    reg(v, 0x048, v->height - 1);
    reg(v, 0x0c0, v->luma.phys + rsz); /* the rings' ends and starts */
    reg(v, 0x0c4, v->luma.phys);
    reg(v, 0x0c8, v->chroma.phys + rsz / 2);
    reg(v, 0x0cc, v->chroma.phys);
    reg(v, 0x0e0, v->width * e->ctb); /* a CTB row, luma and chroma */
    reg(v, 0x0e4, v->width * e->ctb / 2);
    reg(v, 0x0e8, v->width * e->ctb);
    reg(v, 0x0ec, v->width * e->ctb / 2);
    reg(v, 0x0fc, v->width > 1280);
  }
}

int xmv200_venc_create(struct xmv200_venc **pv, uint32_t width,
                       uint32_t height, uint32_t fps, uint32_t qp,
                       uint32_t gop) {
  struct xmv200_venc *v = calloc(1, sizeof(*v));
  uint32_t reset = 0x10002;

  if (!v)
    return -1;
  v->fd_hevc = v->fd_mem = -1;
  v->e = engine();
  v->width = width;
  v->height = height;
  v->hc = (height + 31) & ~31u;
  v->fps = fps ? fps : 25;
  v->qp = qp ? qp : 35;
  v->qp_i = v->qp + v->e->qp_i_up;
  v->gop = gop ? gop : 2 * v->fps;
  v->rc.mode = XMV200_RC_FIXQP;
  v->rc.qp = v->qp;
  v->rc.qp_min = 16;
  v->rc.qp_max = 48;
  v->i_over_p = 4;
  v->fd_hevc = open("/dev/h265", O_RDWR);
  v->fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
  if (v->fd_hevc < 0 || v->fd_mem < 0 ||
      xmv200_mmz_alloc(&v->luma, "vencWrapLuma", ring(v), 0) ||
      xmv200_mmz_alloc(&v->chroma, "vencWrapChroma", ring(v) / 2, 0) ||
      xmv200_mmz_alloc(&v->stream, "stream_chn", lsize(v) / 4 + HDR_ROOM, 0))
    goto fail;
  /* the slice data read back uncached */
  v->stream_map = mmap(NULL, v->stream.size, PROT_READ, MAP_SHARED, v->fd_mem,
                       v->stream.phys);
  if (v->stream_map == MAP_FAILED)
    goto fail;
  ioctl(v->fd_hevc, HEVC_RESET, &reset);
  if (v->e->map == MAP_V1)
    for (int i = 0; i < N(v1_rd); i++)
      xmv200_reg_write(0x21000000 + v1_rd[i].off, v1_rd[i].val);
  venc_regs(v);
  *pv = v;
  return 0;
fail:
  xmv200_venc_destroy(v);
  return -1;
}

int xmv200_venc_create_h264(struct xmv200_venc **pv, uint32_t width,
                            uint32_t height, uint32_t fps, uint32_t qp,
                            uint32_t gop) {
  struct xmv200_venc *v = calloc(1, sizeof(*v));

  if (!v)
    return -1;
  v->fd_hevc = v->fd_mem = -1;
  v->e = &h1_engine;
  v->width = width;
  v->height = height;
  v->fps = fps ? fps : 25;
  v->qp = qp ? qp : 35;
  v->qp_i = v->qp;
  v->gop = gop ? gop : 2 * v->fps;
  v->rc.mode = XMV200_RC_FIXQP;
  v->rc.qp = v->qp;
  v->rc.qp_min = 16;
  v->rc.qp_max = 48;
  v->i_over_p = 4;
  v->h264 = xmv200_h264_create(width, height, v->fps);
  if (!v->h264) {
    free(v);
    return -1;
  }
  *pv = v;
  return 0;
}

void xmv200_venc_destroy(struct xmv200_venc *v) {
  if (!v)
    return;
  xmv200_h264_destroy(v->h264);
  if (v->stream_map && v->stream_map != MAP_FAILED)
    munmap(v->stream_map, v->stream.size);
  if (v->fd_hevc >= 0)
    close(v->fd_hevc);
  if (v->fd_mem >= 0)
    close(v->fd_mem);
  free(v);
}

void xmv200_venc_request_idr(struct xmv200_venc *v) { v->frame = 0; }

static uint32_t clampu(uint32_t x, uint32_t lo, uint32_t hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

int xmv200_venc_set_rc(struct xmv200_venc *v,
                       const struct xmv200_venc_rc *rc) {
  struct xmv200_venc_rc r = *rc;

  if (r.mode < XMV200_RC_FIXQP || r.mode > XMV200_RC_VBR ||
      (r.mode != XMV200_RC_FIXQP && !r.kbps))
    return -1;
  r.qp = r.qp ? clampu(r.qp, 1, 51) : 35;
  r.qp_min = r.qp_min ? clampu(r.qp_min, 1, 51) : 16;
  r.qp_max = r.qp_max ? clampu(r.qp_max, r.qp_min, 51) : 48;
  if (r.qp_max < r.qp_min)
    r.qp_max = r.qp_min;
  if ((r.gop && r.gop != v->gop) || (r.fps && r.fps != v->fps)) {
    if (r.gop)
      v->gop = r.gop;
    if (r.fps)
      v->fps = r.fps;
    if (v->h264)
      xmv200_h264_set_fps(v->h264, v->fps);
    xmv200_venc_request_idr(v); /* the parameter sets carry the rate */
  }
  r.gop = v->gop;
  r.fps = v->fps;
  if (r.mode != v->rc.mode || r.mode == XMV200_RC_FIXQP)
    v->qp = clampu(r.qp, r.qp_min, r.qp_max);
  v->qp_i = clampu(v->qp + v->e->qp_i_up, 1, 51);
  if (r.mode != v->rc.mode)
    v->fill = 0;
  v->rc = r;
  return 0;
}

void xmv200_venc_get_stats(const struct xmv200_venc *v,
                           struct xmv200_venc_stats *st) {
  uint64_t sum = 0;
  uint32_t n = v->fps < 64 ? v->fps : 64;

  for (uint32_t i = 0; i < n; i++)
    sum += v->win_bytes[i];
  st->qp = v->qp;
  st->qp_i = v->qp_i;
  st->kbps = (uint32_t)(sum * 8 / 1000);
  st->frames = v->frames;
  st->bytes = v->bytes;
}

/* After a picture of bits: the next P pictures' QP, from how far the P
 * pictures run over their share of the rate and how much the buffer holds
 * (drained over a GOP); the IDRs follow at the engine's step above. */
static void rate_control(struct xmv200_venc *v, int idr, uint32_t bytes) {
  const struct xmv200_venc_rc *rc = &v->rc;
  double bits = bytes * 8.0, second, budget, share, want, r;
  int step = 0;
  uint32_t lo;

  v->frames++;
  v->bytes += bytes;
  v->win_bytes[v->frames % (v->fps < 64 ? v->fps : 64)] = bytes;
  if (rc->mode == XMV200_RC_FIXQP)
    return;
  second = rc->kbps * 1000.0;
  budget = second / v->fps;
  v->fill += bits - budget;
  if (v->fill > second)
    v->fill = second;
  /* VBR saves nothing up for later */
  if (v->fill < (rc->mode == XMV200_RC_VBR ? 0 : -second))
    v->fill = rc->mode == XMV200_RC_VBR ? 0 : -second;
  if (idr) {
    if (v->avg_p > 0)
      v->i_over_p = 0.5 * v->i_over_p + 0.5 * (bits / v->avg_p);
    if (v->i_over_p < 1)
      v->i_over_p = 1;
    if (v->avg_p > 0)
      return;
    v->avg_p = bits / v->i_over_p; /* the first P pictures' guess */
  } else {
    v->avg_p = v->avg_p > 0 ? 0.7 * v->avg_p + 0.3 * bits : bits;
  }
  /* a P picture's share of a GOP's budget, less what to drain */
  share = budget * v->gop / (v->gop - 1 + v->i_over_p);
  want = share - v->fill / v->gop;
  if (want < share / 4)
    want = share / 4;
  r = v->avg_p / want;
  if (r > 1.5)
    step = 2;
  else if (r > 1.12)
    step = 1;
  else if (r < 0.6)
    step = -2;
  else if (r < 0.88)
    step = -1;
  lo = rc->mode == XMV200_RC_VBR ? (rc->qp > rc->qp_min ? rc->qp : rc->qp_min)
                                 : rc->qp_min;
  v->qp = clampu((uint32_t)((int)v->qp + step), lo, rc->qp_max);
  v->qp_i = clampu(v->qp + v->e->qp_i_up, 1, 51);
}

/* Encodes one picture from VPSS into out: parameter sets before an IDR,
 * then the slice. Returns the length, or -1. */
static int encode(struct xmv200_venc *v, const struct xmv200_frame *f,
                  uint8_t *out, int cap) {
  const struct engine *e = v->e;
  uint32_t lsz = lsize(v), rsz = ring(v), band = v->width * RING_LINES;
  uint8_t rbsp[128];
  uint32_t job[18];
  int idr = v->frame % v->gop == 0, n = 0, k, len;
  const uint32_t *preset;

  if (idr) {
    v->poc = 0;
    if ((k = nal(out + n, cap - n, 32, rbsp, write_vps(v, rbsp, 128))) < 0)
      return -1;
    n += k;
    if ((k = nal(out + n, cap - n, 33, rbsp, write_sps(v, rbsp, 128))) < 0)
      return -1;
    n += k;
    if ((k = nal(out + n, cap - n, 34, rbsp, write_pps(rbsp, 128))) < 0)
      return -1;
    n += k;
  }
  k = nal(out + n, cap - n, idr ? 19 : 1, rbsp,
          write_slice_header(v, idr, rbsp, sizeof(rbsp)));
  if (k < 0)
    return -1;
  n += k;

  preset = e->preset[v->width <= 1280][idr ? 0 : v->frame % v->gop == 1 ? 1 : 2];
  for (int i = 0; i < e->n_type; i++)
    reg(v, e->type_off[i], preset[i]);
  /* the reference is the last reconstruction; this one starts 96 lines
   * below it in the ring */
  uint32_t ref = idr ? lsz : v->recon, rec = idr ? 0 : (v->recon + rsz - band) % rsz;
  if (e->map == MAP_V3) {
    reg(v, 0x040, !idr);
    reg(v, 0x044, idr ? v->qp_i : v->qp);
    reg(v, 0x1e0, f->y);
    reg(v, 0x1e4, f->c);
    reg(v, 0x200, v->stream.phys + HDR_ROOM);
    reg(v, 0x1d4, lsz / 4);
    reg(v, 0x240, ref);
    reg(v, 0x244, ref / 2);
    reg(v, 0x248, rec);
    reg(v, 0x24c, rec / 2);
  } else if (e->map == MAP_X533) {
    reg(v, 0x040, !idr);
    reg(v, 0x044, idr ? v->qp_i : v->qp);
    reg(v, 0x1e0, f->y);
    reg(v, 0x1e4, f->c);
    reg(v, 0x224, f->stride);
    reg(v, 0x228, f->stride);
    reg(v, 0x210, v->stream.phys + HDR_ROOM);
    reg(v, 0x1d4, lsz / 4);
    reg(v, 0x1e8, v->luma.phys + ref);
    reg(v, 0x1ec, v->chroma.phys + ref / 2);
    reg(v, 0x1f0, v->luma.phys + rec);
    reg(v, 0x1f4, v->chroma.phys + rec / 2);
  } else {
    reg(v, 0x054, !idr);
    reg(v, 0x05c, idr ? v->qp_i : v->qp);
    reg(v, 0x0a0, f->y);
    reg(v, 0x0a4, f->c);
    reg(v, 0x0d8, f->stride);
    reg(v, 0x0dc, f->stride);
    reg(v, 0x100, f->width - 1); /* the engine scales the source */
    reg(v, 0x104, f->height - 1);
    reg(v, 0x0d0, v->stream.phys + HDR_ROOM);
    reg(v, 0x088, lsz / 4);
    reg(v, 0x0a8, v->luma.phys + ref);
    reg(v, 0x0ac, v->chroma.phys + ref / 2);
    reg(v, 0x0b0, v->luma.phys + rec);
    reg(v, 0x0b4, v->chroma.phys + rec / 2);
  }

  memset(job, 0, sizeof(job));
  job[e->list_word] = (uint32_t)(uintptr_t)v->regs;
  if (ioctl(v->fd_hevc, HEVC_LOCK, 0) < 0)
    return -1;
  if (ioctl(v->fd_hevc, e->job_cmd, job) < 0 ||
      ioctl(v->fd_hevc, e->wait_cmd, job) < 0) {
    ioctl(v->fd_hevc, HEVC_UNLOCK, 0);
    return -1;
  }
  ioctl(v->fd_hevc, HEVC_UNLOCK, 0);
  len = job[11];
  if (len <= 0 || (uint32_t)len > v->stream.size - HDR_ROOM || n + len > cap)
    return -1;
  memcpy(out + n, v->stream_map + HDR_ROOM, len);
  v->recon = rec;
  v->frame++;
  v->poc++;
  rate_control(v, idr, n + len);
  return n + len;
}

static int venc_fd = -1;

int xmv200_frame_get(struct xmv200_frame *f) {
  struct venc_rec r;

  if (venc_fd < 0)
    venc_fd = open("/dev/venc_info", O_RDWR);
  if (venc_fd < 0 || ioctl(venc_fd, VENC_GET_FRAME, &r) < 0)
    return -1;
  f->chn = r.chn;
  f->y = r.y;
  f->c = r.y + r.stride * r.height;
  f->width = r.width;
  f->height = r.height;
  f->stride = r.stride;
  f->pts = (uint64_t)r.pts_hi << 32 | r.pts_lo;
  memcpy(f->priv, &r, sizeof(r));
  return 0;
}

void xmv200_frame_release(struct xmv200_frame *f) {
  ioctl(venc_fd, VENC_RELEASE_FRAME, f->priv);
}

int xmv200_venc_encode(struct xmv200_venc *v, const struct xmv200_frame *f,
                       uint8_t *out, int cap) {
  if (v->h264) {
    int idr = v->frame % v->gop == 0,
        len = xmv200_h264_encode(v->h264, f, idr, idr ? v->qp_i : v->qp, out,
                                 cap);

    if (len < 0)
      return -1;
    v->frame++;
    rate_control(v, idr, len);
    return len;
  }
  /* v3 and x533 encode what VPSS (or VI) made; v1 scales any source to the
   * picture */
  if (v->e->map != MAP_V1 && (f->width != v->width || f->height != v->height))
    return -1;
  return encode(v, f, out, cap);
}
