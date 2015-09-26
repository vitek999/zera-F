#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/reboot.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <cutils/properties.h>
#include <unistd.h>

#include "common.h"
#include "ftm.h"
#include "miniui.h"
#include "utils.h"

#if 1//def FACTORY_CKT_STANDARD

#define BUFSZ      2048
#define TAG 	   "[CHECK] "

extern int getBarcode(int fd,char *result);

/************check modem calibartion start pengliu 2012.11.26********************/
/*
*  return 0 没有校准
*   	  1 校准成功
*   	  2 校准失败
*/
int check_modem_calibration(void)
{
	int fd = -1;
	int len = 0;
	unsigned int i;
	char barcode[128] = "unknown";
	char* ptr = NULL, * pstr = NULL;
	int reslt;

	fd = openDevice();
	if (-1 == fd)
	{
		LOGD(TAG "Fail to open CCCI interface\n");
		return 0;
	}

	//for (i = 0; i < 30; i++)
	//	usleep(50000); //sleep 1s wait for modem bootup
	send_at(fd, "AT\r\n");
	wait4_ack(fd, NULL, 3000);

	reslt = getBarcode(fd, barcode);
	ptr = strchr(barcode, '\"');
	if (ptr != NULL)
	{
		*ptr = 0;
	}

	closeDevice(fd);

	len = strlen(barcode);
	if (len < 62)
	{
		LOGD(TAG "pengliu have no modem calibration\n");
		reslt = 0;
		return reslt;
	}

	if ('1' == barcode[60] && '0' == barcode[61])
	{
		LOGD(TAG "pengliu calibration pass\n");
		reslt = 1;
	}
	else if ('0' == barcode[60] && '1' == barcode[61])
	{
		LOGD(TAG "pengliu calibration failed\n");
		reslt = 2;
	}
	else
	{
		LOGD(TAG "pengliu unknown\n");
		reslt = 3;
	}
	return reslt;
}
/************check modem calibartion end pengliu 2012.11.26********************/

#endif	/* FACTORY_CKT_STANDARD */

