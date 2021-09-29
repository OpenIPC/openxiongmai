#ifndef RE_DEFINES_H
#define RE_DEFINES_H

#define BITMSK(bit) (XM_S32)(1 << (bit))

#define LINE_LEN_BIT 5
#define LINE_LEN (1 << LINE_LEN_BIT)
#define LINE_BASE_MASK (~(LINE_LEN - 1))

/* Sys */
#define DEFAULT_ALIGN 16
#define MAX_MMZ_NAME_LEN 16

#define MAX_NODE_NUM 16

/* VDA */
#define VDA_MAX_NODE_NUM 32
#define VDA_MAX_INTERNAL 256
#define VDA_CHN_NUM_MAX 32
#define VDA_MAX_WIDTH 960
#define VDA_MAX_HEIGHT 576

/* H264 H265 JPEG */
#define VENC_MAX_CHN_NUM 4

/* Region */
#define OVERLAY_MAX_NUM 8
#define COVER_MAX_NUM 32
#define RGN_MAX_CHN_NUM 3

/* number of channels and device on video input unit of chip
 * Note! VIU_MAX_CHN_NUM is NOT equal to VIU_MAX_DEV_NUM
 * multiplied by VIU_MAX_CHN_NUM, because all VI devices
 * can't work at mode of 4 channles at the same time.
 */
#define VIU_MAX_CHN_NUM 3

#define AIO_MAX_CHN_NUM 1
#define AENC_MAX_CHN_NUM 1
#define ADEC_MAX_CHN_NUM 1

#define AI_DEV_MAX_NUM 1
#define AO_DEV_MIN_NUM 0
#define AO_DEV_MAX_NUM 1

#endif /* RE_DEFINES_H */
