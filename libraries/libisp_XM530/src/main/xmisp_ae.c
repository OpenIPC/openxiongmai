/* libxmisp's auto-exposure (MIT), SoC- and sensor-independent, built into
 * this library, not exported: the open AE (isp_ae.c) drives it */
#include <math.h>
#include <stdint.h>

#pragma GCC visibility push(hidden)
#include "../../../libxmisp/ae.c"
#pragma GCC visibility pop
