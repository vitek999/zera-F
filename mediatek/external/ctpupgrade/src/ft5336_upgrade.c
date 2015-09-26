/******************* V:\szgit\92jb9\mediatek\external\ctpupgrade\src\ft5336_upgrade.c ************************
    创建时间: 2013年11月19日  10:05:25
    主要功能: 
    作    者: 苏 勇 
    版    本: 初始版本   1.0
    其    它:       
******************************************************************/
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>

// For Firmware Update
#define TOUCH_IOC_MAGIC 'A'
#define TPD_UPGRADE_CKT _IO(TOUCH_IOC_MAGIC, 2)
static const char	*touch_dev = "/dev/ft5336";
static int useage(void)
{
	printf("this app is upgdrade ctp fireware form *.i file\n");
	printf("useage:\n");
	printf("ft5336_upgrade upgradefile.i\n");
	return 0;
}
#define MAX_TRY (3)
int main(int argc, char **argv)
{
	int re = -1;
	char *ptouchdata = NULL;
	int size;
	int fd;
	int ctpfile;
	int i=0;
	if(argc == 2)
	{
		fd = open(touch_dev, O_RDONLY);
		if(fd < 0)
		{
			printf("Couldn't open '%s' (%s)\n", touch_dev, strerror(errno));
			return -1;
		}

		ctpfile = open(argv[1], O_RDONLY);
		if(ctpfile < 0)
		{
			printf("Couldn't open '%s' (%s)\n", argv[1], strerror(errno));
			re = -2;
			goto FDERROR;
		}

		size = lseek(ctpfile, 0, SEEK_END);
		ptouchdata = malloc(size+sizeof(int));
		if(ptouchdata == NULL)
		{
			re = -3;
			goto CTPERROR;
		}

		lseek(ctpfile, 0, SEEK_SET);
		int len = read(ctpfile, ptouchdata+sizeof(int), size);
		
		if(len != size)
		{
			printf("read error %s\n", argv[1]);
			re = -3;
			goto MALLOCERROR;
		}
		*((int *)ptouchdata)=size;
		printf("upgrade ...\n");

		for(i=0;i<MAX_TRY;i++)
		{
			int err = ioctl(fd, TPD_UPGRADE_CKT, ptouchdata);
			if(err)
			{
				printf("error TPD_UPGRADE_CKT %d %d\n", i,err);
			}
			else
			{
				printf("done\n");
				break;
			}
		}
	}
	else
	{
		useage();
		return 0;
	}

MALLOCERROR:
	free(ptouchdata);
	ptouchdata=NULL;
CTPERROR:
	close(ctpfile);
FDERROR:
	close(fd);

	return re;
}
