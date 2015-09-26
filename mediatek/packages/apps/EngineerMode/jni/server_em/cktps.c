/*
* Source code of daemon pscalid
* This daemon is run one time at Android MS bootup to set the ps threshold
*
* This file should locate at /mediatek/source/hardware/sensor/pscalid
*
* Configure the following file to start the daemon
* mediatek/config/ckt73_gb/init.rc
*
* Version History
*
* 2011-09-22 Jason J. Inital version
*
*/

#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <strings.h>


#include "../../../external/nvram/libnvram/libnvram.h"
#include "../../../custom/common/cgen/inc/CFG_file_lid.h"
#include "../../../custom/ckt82_we_kk/cgen/cfgfileinc/CFG_PRODUCT_INFO_File.h"
#include "../../../custom/ckt82_we_kk/cgen/inc/Custom_NvRam_LID.h"

//extern bool NVM_CloseFileDesc(F_ID iFileID);
//The following definition is from mediatek/source/kernel/include/linux/sensors_io.h
//We put it here in order to make compilation success
#include <linux/sensors_io.h>
//extern F_ID NVM_GetFileDesc(int file_lid, int *pRecSize, int *pRecNum, bool IsRead);

#include "cktps.h"

static const char* alsps_dev="/dev/als_ps";

#define TAG "CKT_PS"
#define PSCALILOGD(fmt,args...) printf(TAG "%s %d: "fmt,__FUNCTION__,__LINE__,##args)
PRODUCT_INFO test_struct;


int read_offdata(int *data)
{
	int result;
	F_ID test_nvram_id;
	int file_lid = AP_CFG_REEB_PRODUCT_INFO_LID;
	int test_fd;
	int rec_size=0 , rec_num = 0;
	bool IsRead = true,IsWrite = false;

	*data=0;
	PSCALILOGD("--@read:\n");
	test_nvram_id = NVM_GetFileDesc(file_lid, &rec_size, &rec_num,IsRead);//IsRead=true
	if(test_nvram_id.iFileDesc < 0){
		PSCALILOGD("--@NVM_GetFileDesc failed\n");
		return -1;
	}
	result = read(test_nvram_id.iFileDesc,&test_struct,rec_num*rec_size);
	if(result != rec_num*rec_size){
		NVM_CloseFileDesc(test_nvram_id);
		PSCALILOGD("--@Get file failed\n");
		return -1;
	}
	//读取nvram的数据
	//*data = test_struct.trace_nvram_data.info_name_perso1[0];
	//*data = test_struct.ps_cali_nvram_data[0];
	*data = test_struct.ps_cali_nvram_data[0] & 0xff;
	*data = ((test_struct.ps_cali_nvram_data[1] & 0xff) << 8) |*data;
	PSCALILOGD("--@---------read---------\n");
	//PSCALILOGD("--@perso1[0]:%d\n",test_struct.trace_nvram_data.info_name_perso1[0]);
	PSCALILOGD("--@perso1[0]:%d\n",test_struct.ps_cali_nvram_data[0]);
	PSCALILOGD("--@------------------\n");
	if(!NVM_CloseFileDesc(test_nvram_id))
	{
		PSCALILOGD("NVM_CloseFileDesc failed\n");
		return -1;
	}
    return 0;
}


static int write_offdata(int *data)
{
	int result;
	F_ID test_nvram_id;
	int file_lid = AP_CFG_REEB_PRODUCT_INFO_LID;
	int test_fd;
	int rec_size=0 , rec_num = 0;
	bool IsRead = true,IsWrite = false;

	PSCALILOGD("write:\n");
    test_nvram_id = NVM_GetFileDesc(file_lid, &rec_size, &rec_num,IsWrite);//IsWrite=false
    if(test_nvram_id.iFileDesc < 0){
        PSCALILOGD("NVM_GetFileDesc failed\n");
        return -1;
    }
   // test_struct.trace_nvram_data.info_name_perso1[0] = *data;
      //  test_struct.ps_cali_nvram_data[0] = (char)*data;
      		test_struct.ps_cali_nvram_data[0] = *data & 0xff;
		test_struct.ps_cali_nvram_data[1] = (*data & 0xff00) >> 8;
    PSCALILOGD("--------write----------\n");
	//PSCALILOGD("--@perso1[0]:%d\n",test_struct.trace_nvram_data.info_name_perso1[0]);
	PSCALILOGD("--@perso1[0]:%d\n",test_struct.ps_cali_nvram_data[0]);
	PSCALILOGD("--@------------------\n");
    result = write(test_nvram_id.iFileDesc,&test_struct,rec_num*rec_size);
    if(result != rec_num*rec_size){
    	NVM_CloseFileDesc(test_nvram_id);
        PSCALILOGD("write file failed\n");
        return -1;
    }
    if(!NVM_CloseFileDesc(test_nvram_id))
    {
        PSCALILOGD("NVM_CloseFileDesc failed\n");
		return -1;
    }
    return 0;
}


int get_ps_data(int *pdata)
{
	int re=-1;
    int fd = open("/dev/als_ps", O_RDONLY);
    if (fd < 0)
	{
        PSCALILOGD("Couldn't open '%s' (%s)", alsps_dev, strerror(errno));
        return -1;
    }
    int err = ioctl(fd, ALSPS_GET_PS_RAW_DATA, pdata);
	if(err)
	{
        PSCALILOGD("Failed to set PS offset data: %u", *pdata);
		*pdata=0;
	}
	else
	{
        PSCALILOGD("Set PS offset data: %u done.", *pdata);
		re=0;
	}
	close(fd);

	return re;
}

int do_ps_calibration (int *pdata)
{
	int fd = open("/sys/bus/platform/drivers/als_ps/pscalibrate", O_RDONLY);
	char xbuf[PAGE_SIZE];
	int re=-1;
    if (fd < 0)
	{
        PSCALILOGD("Couldn't open '%s' (%s)", alsps_dev, strerror(errno));
        return -1;
    }
	*pdata=0;
    int num =read(fd,xbuf,PAGE_SIZE);
	if(num>0)
	{
		if(1 == sscanf(xbuf,"%d",pdata))
		{
			re=0;
		}
	}
	
	close(fd);
	if(re==0)
	{
		re= write_offdata(pdata);
	}
	return re;
}

int clear_ps_calibration (int *pdata)
{
	*pdata=0;
	int re=-1;
	
	int fd = open("/dev/als_ps", O_RDONLY);
    if (fd < 0)
	{
        PSCALILOGD("Couldn't open '%s' (%s)", alsps_dev, strerror(errno));
        return -1;
    }
    int err = ioctl(fd, ALSPS_SET_PS_CALI, pdata);
	if(err)
	{
        PSCALILOGD("Failed to clear PS offset data: %u", *pdata);
	}
	else
	{
        PSCALILOGD("clear PS offset data: %u done.", *pdata);
		re=0;
	}
	
	close(fd);

	if(re==0)
	{
		re= write_offdata(pdata);
	}
	return re;
}


