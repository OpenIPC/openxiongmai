#ifndef RE_COMM_REGION_H
#define RE_COMM_REGION_H

#include "re_comm_video.h"
#include "re_common.h"
#include "re_defines.h"
#include "re_errno.h"

typedef XM_U32 RGN_HANDLE;

/* type of video regions */
typedef enum xmRGN_TYPE_E {
  OVERLAY_RGN = 0, /* video overlay region */
  COVER_RGN,
  COVEREX_RGN,
  OVERLAYEX_RGN,
  RGN_BUTT
} RGN_TYPE_E;

typedef enum xmINVERT_COLOR_MODE_E {
  LESSTHAN_LUM_THRESH = 0, /* the lum of the video is less than the lum
                              threshold which is set by u32LumThresh  */
  MORETHAN_LUM_THRESH, /* the lum of the video is more than the lum threshold
                          which is set by u32LumThresh  */
  INVERT_COLOR_BUTT
} INVERT_COLOR_MODE_E;

typedef struct xmOVERLAY_QP_INFO_S {
  XM_BOOL bAbsQp;
  XM_S32 s32Qp;
} OVERLAY_QP_INFO_S;

typedef struct xmOVERLAY_INVERT_COLOR_S {
  SIZE_S stInvColArea; // It must be multipe of 16 but not more than 64.
  XM_U32 u32LumThresh; // The threshold to decide whether invert the OSD's color
                       // or not.
  INVERT_COLOR_MODE_E enChgMod;
  XM_BOOL bInvColEn; // The switch of inverting color.
} OVERLAY_INVERT_COLOR_S;

/* Define which feild to be attached */
typedef enum xmRGN_ATTACH_FIELD_E {
  RGN_ATTACH_FIELD_FRAME = 0, /* whole frame */
  RGN_ATTACH_FIELD_TOP,       /* top field */
  RGN_ATTACH_FIELD_BOTTOM,    /* bottom field */

  RGN_ATTACH_FIELD_BUTT
} RGN_ATTACH_FIELD_E;

typedef struct xmOVERLAY_COMM_ATTR_S {
  XM_U32
  u32FgColor; // bit27-24:ID,23~16:Y,15~8:U,7~0:V, 同一个ID只能设置一种颜色
  XM_U32 u32BgColor;
  XM_U32 u32Effect; // BIt0-3
                    // 透明度,4-7淡入淡出,8-11闪烁,12-15水平放大,16-19垂直放大
  RECT_S stRect;
} OVERLAY_ATTR_S;

typedef struct xmOVERLAY_CHN_ATTR_S {
  /* X:[0,4096],align:4,Y:[0,4096],align:4 */
  POINT_S stPoint;

  /* background an foreground transparence when pixel format is ARGB1555
   * the pixel format is ARGB1555,when the alpha bit is 1 this alpha is value!
   * range:[0,128]
   */
  XM_U32 u32FgAlpha;

  /* background an foreground transparence when pixel format is ARGB1555
   * the pixel format is ARGB1555,when the alpha bit is 0 this alpha is value!
   * range:[0,128]
   */
  XM_U32 u32BgAlpha;

  XM_U32 u32Layer; /* OVERLAY region layer range:[0,7]*/

  OVERLAY_QP_INFO_S stQpInfo;

  OVERLAY_INVERT_COLOR_S stInvertColor;
} OVERLAY_CHN_ATTR_S;

typedef struct xmCOVER_CHN_ATTR_S {
  RECT_S stRect;
  XM_U32 u32Color;
  XM_U32 u32Layer; /* COVER region layer range:[0,3] */
} COVER_CHN_ATTR_S;

typedef struct xmCOVEREX_CHN_ATTR_S {
  RECT_S stRect;
  XM_U32 u32Color;
  XM_U32 u32Layer; /* COVEREX region layer range:[0,7] */
} COVEREX_CHN_ATTR_S;

typedef struct xmOVERLAYEX_COMM_ATTR_S {
  PIXEL_FORMAT_E enPixelFmt;

  /* background color, pixel format depends on "enPixelFmt" */
  XM_U32 u32BgColor;

  /* region size,W:[4,1920],align:2,H:[4,1080],align:2 */
  SIZE_S stSize;
} OVERLAYEX_ATTR_S;

typedef struct xmOVERLAYEX_CHN_ATTR_S {
  /* X:[0,4096],align:4,Y:[0,4636],align:4 */
  POINT_S stPoint;

  /* background an foreground transparence when pixel format is ARGB1555
   * the pixel format is ARGB1555,when the alpha bit is 1 this alpha is value!
   * range:[0,128]
   */
  XM_U32 u32FgAlpha;

  /* background an foreground transparence when pixel format is ARGB1555
   * the pixel format is ARGB1555,when the alpha bit is 0 this alpha is value!
   * range:[0,128]
   */
  XM_U32 u32BgAlpha;

  XM_U32 u32Layer; /* OVERLAYEX region layer range:[0,15]*/
} OVERLAYEX_CHN_ATTR_S;

typedef struct xmCOVER_COMM_ATTR_S {
  XM_U32 u32Color;
  RECT_S stRect;
} COVER_ATTR_S;

typedef union xmRGN_ATTR_U {
  OVERLAY_ATTR_S stOverlay;
  COVER_ATTR_S stCover;
} RGN_ATTR_U;

typedef union xmRGN_CHN_ATTR_U {
  OVERLAY_CHN_ATTR_S stOverlayChn;     /* attribute of overlay region */
  COVER_CHN_ATTR_S stCoverChn;         /* attribute of cover region */
  COVEREX_CHN_ATTR_S stCoverExChn;     /* attribute of coverex region */
  OVERLAYEX_CHN_ATTR_S stOverlayExChn; /* attribute of overlayex region */
} RGN_CHN_ATTR_U;

/* attribute of a region */
typedef struct xmRGN_ATTR_S {
  XM_U32 u32Handle;
  RGN_TYPE_E enType;
  RGN_ATTR_U unAttr;
} RGN_ATTR_S;

/* attribute of a region */
typedef struct xmRGN_CHN_ATTR_S {
  XM_BOOL bShow;
  RGN_TYPE_E enType;        /* region type */
  RGN_CHN_ATTR_U unChnAttr; /* region attribute */
} RGN_CHN_ATTR_S;

#define RGN_MAX_BMP_UPD_NUM 16

typedef struct xmRGN_BMP_UPD_S {
  POINT_S stPoint;
  BITMAP_S stBmp;
  XM_U32 u32Stride;
} RGN_BMP_UPD_S;

typedef struct xmRGN_BMP_UPD_CFG_S {
  XM_U32 u32BmpCnt;
  RGN_BMP_UPD_S astBmpUpd[RGN_MAX_BMP_UPD_NUM];
} RGN_BMP_UPD_CFG_S;

/* invlalid device ID */
#define XM_ERR_RGN_INVALID_DEVID                                               \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_INVALID_DEVID)

/* invlalid channel ID */
#define XM_ERR_RGN_INVALID_CHNID                                               \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_INVALID_CHNID)

/* at lease one parameter is illagal ,eg, an illegal enumeration value  */
#define XM_ERR_RGN_ILLEGAL_PARAM                                               \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_ILLEGAL_PARAM)

/* channel exists */
#define XM_ERR_RGN_EXIST XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_EXIST)

/*UN exist*/
#define XM_ERR_RGN_UNEXIST                                                     \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_UNEXIST)

/* using a NULL point */
#define XM_ERR_RGN_NULL_PTR                                                    \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_NULL_PTR)

/* try to enable or initialize system,device or channel, before configing
 * attribute */
#define XM_ERR_RGN_NOT_CONFIG                                                  \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_NOT_CONFIG)

/* operation is not supported by NOW */
#define XM_ERR_RGN_NOT_SUPPORT                                                 \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_NOT_SUPPORT)

/* operation is not permitted ,eg, try to change stati attribute */
#define XM_ERR_RGN_NOT_PERM                                                    \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_NOT_PERM)

/* failure caused by malloc memory */
#define XM_ERR_RGN_NOMEM XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_NOMEM)

/* failure caused by malloc buffer */
#define XM_ERR_RGN_NOBUF XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_NOBUF)

/* no data in buffer */
#define XM_ERR_RGN_BUF_EMPTY                                                   \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_BUF_EMPTY)

/* no buffer for new data */
#define XM_ERR_RGN_BUF_FULL                                                    \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_BUF_FULL)

/* bad address, eg. used for copy_from_user & copy_to_user */
#define XM_ERR_RGN_BADADDR                                                     \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_BADADDR)

/* resource is busy, eg. destroy a venc chn without unregistering it */
#define XM_ERR_RGN_BUSY XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_BUSY)

/* System is not ready,maybe not initialed or loaded.
 * Returning the error code when opening a device file failed.
 */
#define XM_ERR_RGN_NOTREADY                                                    \
  XM_DEF_ERR(XM_ID_RGN, EN_ERR_LEVEL_ERROR, EN_ERR_SYS_NOTREADY)

#endif /* RE_COMM_REGION_H */
