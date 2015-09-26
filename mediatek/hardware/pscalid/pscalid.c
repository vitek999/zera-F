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


#include "../../external/nvram/libnvram/libnvram.h"
#include "../../custom/common/cgen/inc/CFG_file_lid.h"
#include "../../custom/ckt82_we_kk/cgen/cfgfileinc/CFG_PRODUCT_INFO_File.h"
#include "../../custom/ckt82_we_kk/cgen/inc/Custom_NvRam_LID.h"
//extern bool NVM_CloseFileDesc(F_ID iFileID);
//The following definition is from mediatek/source/kernel/include/linux/sensors_io.h
//We put it here in order to make compilation success
#include <linux/sensors_io.h>
//extern F_ID NVM_GetFileDesc(int file_lid, int *pRecSize, int *pRecNum, bool IsRead);

#define ALSPS							0X84
#define ALSPS_SET_PS_MODE					_IOW(ALSPS, 0x01, int)
#define ALSPS_GET_PS_MODE					_IOR(ALSPS, 0x02, int)
#define ALSPS_GET_PS_DATA					_IOR(ALSPS, 0x03, int)
#define ALSPS_GET_PS_RAW_DATA				_IOR(ALSPS, 0x04, int)
#define ALSPS_SET_ALS_MODE					_IOW(ALSPS, 0x05, int)
#define ALSPS_GET_ALS_MODE					_IOR(ALSPS, 0x06, int)
#define ALSPS_GET_ALS_DATA					_IOR(ALSPS, 0x07, int)
#define ALSPS_GET_ALS_RAW_DATA           	_IOR(ALSPS, 0x08, int)
#define ALSPS_SET_PS_CALI           	    _IOR(ALSPS, 0x17, int)
// xiangfei.peng 20140513 because _IOR(ALSPS, 0x09, int) is already defined for ALSPS_GET_PS_TEST_RESULT

static const char* ps_cali_file="/data/data/com.ckt.pscali_data";
static const char* alsps_dev="/dev/als_ps";

#define TAG "[PSCALID] "
#define PSCALILOGD(fmt,args...) printf(TAG "%s %d: "fmt,__FUNCTION__,__LINE__,##args)
PRODUCT_INFO test_struct;

static int read_offdata(unsigned int *data)
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

int main(int argc, char** argv){

	unsigned int ps_offset=0;
	unsigned int offset_data=0;
	if(read_offdata(&offset_data))
		PSCALILOGD("Read calibration file failed!\n");
	PSCALILOGD("--@offset_data:%d\n",offset_data);
	ps_offset = (unsigned int)offset_data;
	/*
	FILE* fp = fopen(ps_cali_file, "r");
	if(NULL==fp){
		PSCALILOGD("Open calibration file failed! PS has not been calibrated!\n");
		return 1;
	}

        if(EOF==fscanf(fp, "%u", &ps_threshold_hi)){
		PSCALILOGD("Read calibration value from file failed! PS has not been calibrated!\n");
		return 1;
	}

	//TODO: Add debug info
	PSCALILOGD("Read calibration value :%d\n",ps_threshold_hi);

        fclose(fp);

	//TODO: Set threshold via ioctl
	*/
	//ps_threshold_hi = (unsigned int)offset_data;
	if(ps_offset>=0)
	{
        int fd = open("/dev/als_ps", O_RDONLY);
        if (fd < 0)
		{
            PSCALILOGD("Couldn't open '%s' (%s)", alsps_dev, strerror(errno));
            return -1;
        }
        int err = ioctl(fd, ALSPS_SET_PS_CALI, &ps_offset);
		if(err)
		{
            PSCALILOGD("Failed to set PS offset data: %u", ps_offset);
		}
		else
		{
            PSCALILOGD("Set PS offset data: %u done.", ps_offset);
		}
	   close(fd);
	}
	else
	{
		PSCALILOGD("ps_offset value in calibration file is not correct or cleared, skip the calibration!\n");
	}

	return 0;
}
