#ifndef RE_COMM_SYS_H
#define RE_COMM_SYS_H

#include "re_debug.h"
#include "re_errno.h"
#include "re_type.h"

#define XM_TRACE_SYS(level, fmt...) XM_TRACE(level, XM_ID_SYS, ##fmt)
typedef struct xmMPP_SYS_CONF_S {
  /* stride of picture buffer must be aligned with this value.
   * you can choose a value from 1 to 1024, and it must be multiple of 16.
   */
  XM_U32 u32AlignWidth;

} MPP_SYS_CONF_S;

#define XM_ERR_SYS_NULL_PTR                                                    \
  XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_NULL_PTR)
#define XM_ERR_SYS_NOTREADY                                                    \
  XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_SYS_NOTREADY)
#define XM_ERR_SYS_NOT_PERM                                                    \
  XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_NOT_PERM)
#define XM_ERR_SYS_NOMEM XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_NOMEM)
#define XM_ERR_SYS_ILLEGAL_PARAM                                               \
  XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_ILLEGAL_PARAM)
#define XM_ERR_SYS_BUSY XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_BUSY)
#define XM_ERR_SYS_NOT_SUPPORT                                                 \
  XM_DEF_ERR(XM_ID_SYS, EN_ERR_LEVEL_ERROR, EN_ERR_NOT_SUPPORT)

#endif /* RE_COMM_SYS_H */
