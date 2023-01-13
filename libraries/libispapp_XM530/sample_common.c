#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "Camera.h"
#include "mpi_isp.h"
#include "mpi_phyvi.h"
#include "xm_common.h"
#include "xm_defines.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"
#include "xm_type.h"

#define XM_INFO "/proc/xm/xminfo"
#define FORMAT_FILE "/mnt/mtd/Config/Json/format"
#define SNSTYPE_FILE "/mnt/mtd/Config/SensorType.bat"

extern XM_S32 Mipi_check_init(MIPI_CK_CMD *ptMpck);
extern XM_S32 Mipi_check_run(void);

int string_get_value(const char *filename, const char *str, const char separate,
                     char *value, char *raw) {
#define BYTES_PER_LINE 128
  FILE *fp;
  char line[BYTES_PER_LINE];
  char tmpstr[BYTES_PER_LINE];
  char *pos;

  if (strlen(str) > BYTES_PER_LINE - 1) {
    ERR("strlen=%d\n", strlen(str));
    return XM_FAILURE;
  }
  if (NULL == (fp = fopen(filename, "r"))) {
    // ERR("open %s failed\n", filename);
    return XM_FAILURE;
  }
  while (fgets(line, sizeof(line), fp)) /*lager than BYTES_PER_LINE is bug*/
  {
    pos = strstr(line, str);
    if (NULL != pos) {
      sprintf(tmpstr, "%s%c", str, separate);
      pos += strlen(tmpstr);
      if (NULL != raw) {
        if (strlen(pos) > BYTES_PER_LINE)
          strncpy(raw, pos, BYTES_PER_LINE);
        else
          strncpy(raw, pos, strlen(pos));
      }
      if (NULL != value) {
        sscanf(pos, "%s", value);
      }
      fclose(fp);
      return XM_SUCCESS;
    }
  }
  fclose(fp);
  return XM_FAILURE;
}

XM_U8 xmprop_get_value_v2(XM_U8 *KeyName, XM_VOID *pu32Addr) {
  XM_U8 u8ret = 0;
  XM_U8 value[64] = {0};
  // 1. xmuart 使能开关
  if (!strcmp((const char *)"xmuart", (const char *)KeyName)) {
    if (string_get_value(XM_INFO, (const char *)KeyName, ':', (char *)value,
                         NULL) != XM_SUCCESS) {
      DEBUG("[xmprop] [%s] is not exist!\n", KeyName);
      u8ret = 0;
      return u8ret;
    }
    u8ret = 1;
    *((XM_U8 *)pu32Addr) = 0;
    if (!strcmp("1", (const char *)value)) {
      *((XM_U8 *)pu32Addr) = 1;
    }
    DEBUG("[%s] %d\n", KeyName, *((XM_U8 *)pu32Addr));
  }
#if 0
	// 2. PAL/NTSC
	if(!strcmp((const char *)"vstd",(const char *)KeyName))
	{
		if(string_get_value(FORMAT_FILE,"NTSC",':',(char *)value,NULL) == XM_SUCCESS)
		{
			value[0] = NTSCS;
		}
		else
		{
			value[0] = PALS;
		}
		*((XM_U8*)pu32Addr) = (XM_U8)value[0];
		u8ret = 1;
		DEBUG("[%s] %s\n",KeyName, (value[0]==PALS)?"PAL":"NTSC"); 
	}
#endif

  // 3. sensor type
  if (!strcmp((const char *)"snsType", (const char *)KeyName)) {
    if (string_get_value(SNSTYPE_FILE, "snsType", ':', (char *)value, NULL) ==
        XM_SUCCESS) {
      *((XM_U32 *)pu32Addr) = atoi((char *)value);
    }
  }
  return u8ret;
}

XM_S32 xmprop_set_value_v2(XM_U8 *KeyName, XM_VOID *pu32Addr) {
  FILE *fp;
  // XM_U8 u8ret=0;
  // XM_U8 value[64] = {0};
  //  1. sensor type
  if (!strcmp((const char *)"snsType", (const char *)KeyName)) {
    if (NULL == (fp = fopen(SNSTYPE_FILE, "w"))) {
      ERR("open %s failed\n", SNSTYPE_FILE);
      return XM_FAILURE;
    }
    fprintf(fp, "snsType:%d\n", *((XM_U32 *)pu32Addr));
    fclose(fp);
  }
  return XM_SUCCESS;
}

int Mipi_check(void) {
  MIPI_CK_CMD mipi_ck;
  XM_MPI_ISP_SetRegister(0x300F0040, 0);
  XM_MPI_ISP_SetRegister(0x300F0060, 0);
  XM_MPI_ISP_SetRegister(0x30088000, 0);
  sleep(1);
  mipi_ck.mipi_ck_div = 3;
  mipi_ck.mipi_ck_count = 200;
  mipi_ck.mipi_ck_all_v = 0;
  mipi_ck.mipi_ck_all_h = 0;
  mipi_ck.mipi_ck_en = 1;
  Mipi_check_init(&mipi_ck);
  sleep(1);
  Mipi_check_run();
  return 0;
}
