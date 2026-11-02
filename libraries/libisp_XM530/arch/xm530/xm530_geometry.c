/* What the vendor library writes for the picture size, found by tracing it
 * with every window from 1856 to 2600 wide and 1056 to 1954 high in
 * qemu-bench (tools/xm-isp-trace/start530.py checks it on every trace). */
#include "xm530_start.h"

static XM_U32 cdiv(XM_U32 a, XM_U32 b) { return (a + b - 1) / b; }

void xm530_planes(XM_U32 w, XM_U32 h, struct xm530_planes *p) {
  p->frame = cdiv(w, 5) * 8 * h;
  p->motion = cdiv(w, 10) * 4 * h;
  p->err_w = cdiv(w, 16) * 8;
  p->err_h = h / 2;
  p->err = p->err_w * p->err_h;
}

void xm530_geometry(XM_U32 w, XM_U32 h, XM_U32 mmz8,
                    XM_U32 geo[XM530_GEO_NUM]) {
  struct xm530_planes p;
  xm530_planes(w, h, &p);
  const XM_U32 nx = (w + 16) / 32 - 1, ny = (h + 16) / 32 - 1;
  const XM_U32 kx = w - 31 * (nx + 1), ky = h - 31 * (ny + 1);
  const XM_U32 m = 1u << 20;
  geo[XM530_GEO_NR3D_MOTION] = mmz8 + p.frame / 8;
  geo[XM530_GEO_NR3D_ERR] = mmz8 + (p.frame + p.motion) / 8;
  geo[XM530_GEO_NR3D_FRAME_LINE] = cdiv(w, 5);
  geo[XM530_GEO_NR3D_MOTION_LINE] = cdiv(w, 10);
  geo[XM530_GEO_NR3D_ERR_LINE] = cdiv(w, 16);
  geo[XM530_GEO_LTM_NX] = nx;
  geo[XM530_GEO_LTM_NY] = ny;
  geo[XM530_GEO_LTM_STEP_X] = m / (nx + 1);
  geo[XM530_GEO_LTM_STEP_Y] = m / (ny + 1);
  geo[XM530_GEO_LTM_TL] = m / ((nx + 1) * (ny + 1));
  geo[XM530_GEO_LTM_TR] = m / (kx * (ny + 1));
  geo[XM530_GEO_LTM_BL] = m / ((nx + 1) * ky);
  geo[XM530_GEO_LTM_BR] = m / (kx * ky);
}
