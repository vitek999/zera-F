/* Copyright Statement:
 *
 * Edit History
 * 2011-09-21 Jason J. Inital version
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/mount.h>
#include <sys/statfs.h>
#include <dirent.h>
#include <linux/input.h>
#include <math.h>
#include <sys/file.h>

#include "common.h"
#include "miniui.h"
#include "ftm.h"


#include "libnvram.h"
#include "../../../custom/ckt82_we_kk/cgen/cfgfileinc/CFG_PRODUCT_INFO_File.h"
#include "../../../custom/ckt82_we_kk/cgen/inc/Custom_NvRam_LID.h"
#ifdef CUSTOM_KERNEL_ALSPS
#include <linux/sensors_io.h>
/******************************************************************************
 * MACRO
 *****************************************************************************/
#define TAG "[PSCALI] "
#define mod_to_lps_data(p) (struct lps_data*)((char*)(p) + sizeof(struct ftm_module))
#define FLPLOGD(fmt, arg ...) LOGD(TAG fmt, ##arg)
#define FLPLOGE(fmt, arg ...) LOGE("%s [%5d]: " fmt, __func__, __LINE__, ##arg)


//#define ALSPS_GET_PS_CALI                   _IOR(ALSPS, 0x0F, int)
//#define ALSPS_GET_ID                        _IOR(ALSPS, 0x10, int)
//#define ALSPS_RESET_PS                      _IOR(ALSPS, 0x11, int)
#define ALSPS_GET_PS_CALI                   _IOR(ALSPS, 0x14, int)
#define ALSPS_GET_ID                        _IOR(ALSPS, 0x15, int)
#define ALSPS_RESET_PS                      _IOR(ALSPS, 0x16, int)
#define ALSPS_SET_PS_CALI           	    _IOR(ALSPS, 0x17, int)
// xiangfei.peng 20140513 because _IOR(ALSPS, 0x09, int) is already defined for ALSPS_GET_PS_TEST_RESULT
PRODUCT_INFO test_struct;
/******************************************************************************
 * Structure
 *****************************************************************************/
enum {
    ITEM_CLEAR,
    ITEM_DOCALI,
    ITEM_EXIT,
};
/*---------------------------------------------------------------------------*/
enum{
    PS_OP_NONE,
    PS_OP_CLEAR,
    PS_OP_CALI_PRE,
    PS_OP_CALI,
};

static item_t ps_cali_items[] = {
    item(ITEM_CLEAR,  "Clear Calibration"),
    item(ITEM_DOCALI, "Do Calibration"),
    item(ITEM_EXIT,   "Exit"),
    item(-1, NULL),
};

/*---------------------------------------------------------------------------*/
struct lps_priv
{
    /*specific data field*/
    char    *dev;
    int     fd;
    unsigned int als_raw;
    unsigned int ps_raw;

    pthread_mutex_t evtmutex;
    int  pending_op;
    int  cali_delay;
    int  cali_num;

    char status[1024];
};
/*---------------------------------------------------------------------------*/
struct lps_data
{
    struct lps_priv lps;

    /*common for each factory mode*/
    char  info[1024];
    bool  avail;
    bool  exit_thd;

    text_t    title;
    text_t    text;
    text_t    left_btn;
    text_t    center_btn;
    text_t    right_btn;

    pthread_t update_thd;
    struct ftm_module *mod;
    struct textview tv;
    struct itemview *iv;
};
bool bUpToDate; 
static int read_offset(unsigned int *data)
{
	int result;
	F_ID test_nvram_id;
	int file_lid = AP_CFG_REEB_PRODUCT_INFO_LID;
	int test_fd;
	int rec_size=0 , rec_num = 0;
	bool IsRead = true,IsWrite = false;
//	PSCALILOGD("--@read:\n");
	test_nvram_id = NVM_GetFileDesc(file_lid, &rec_size, &rec_num,IsRead);//IsRead=true
	if(test_nvram_id.iFileDesc < 0){
	//	PSCALILOGD("--@NVM_GetFileDesc failed\n");
		return -1;
	}
	result = read(test_nvram_id.iFileDesc,&test_struct,rec_num*rec_size);
	if(result != rec_num*rec_size){
	//	PSCALILOGD("--@Get file failed\n");
		return -1;
	}
	//*data = test_struct.ps_cali_nvram_data[0];
	
	//*data = test_struct.trace_nvram_data.info_name_perso1[0];
	*data = test_struct.ps_cali_nvram_data[0] & 0xff;
	*data = ((test_struct.ps_cali_nvram_data[1] & 0xff) << 8) |*data;
//	PSCALILOGD("--@---------read---------\n");
	//PSCALILOGD("--@perso1[0]:%d\n",test_struct.trace_nvram_data.info_name_perso1[0]);
	//PSCALILOGD("--@------------------\n");
	if(!NVM_CloseFileDesc(test_nvram_id))
	{
		//PSCALILOGD("NVM_CloseFileDesc failed\n");
		return -1;
	}
    return 0;
}
/******************************************************************************
 * Functions
 *****************************************************************************/
static int read_write(unsigned int *data)
{
	int result;
	F_ID test_nvram_id;
	int file_lid = AP_CFG_REEB_PRODUCT_INFO_LID;
	int test_fd;
	int rec_size=0 , rec_num = 0;
	bool IsRead = true,IsWrite = false;

	FLPLOGD("--@read:\n");
	test_nvram_id = NVM_GetFileDesc(file_lid, &rec_size, &rec_num,IsRead);//IsRead=true
	if(test_nvram_id.iFileDesc < 0){
		FLPLOGE("--@NVM_GetFileDesc failed\n");
		return -1;
	}
	result = read(test_nvram_id.iFileDesc,&test_struct,rec_num*rec_size);
	if(result != rec_num*rec_size){
		FLPLOGE("--@Get file failed\n");
		return -1;
	}
	//读取nvram的数据
	FLPLOGD("--@---------read---------\n");
	//FLPLOGD("--@perso1[0]:%d\n",test_struct.trace_nvram_data.info_name_perso1[0]);
	FLPLOGD("--@perso1[0]:%d\n",test_struct.ps_cali_nvram_data[0]);
	FLPLOGD("--@------------------\n");
	if(!NVM_CloseFileDesc(test_nvram_id))
	{
		FLPLOGE("NVM_CloseFileDesc failed\n");
	}

	FLPLOGD("write:\n");
    test_nvram_id = NVM_GetFileDesc(file_lid, &rec_size, &rec_num,IsWrite);//IsWrite=false
    if(test_nvram_id.iFileDesc < 0){
        FLPLOGE("NVM_GetFileDesc failed\n");
        return -1;
    }
       // test_struct.ps_cali_nvram_data[0] = *data;
   // test_struct.trace_nvram_data.info_name_perso1[0] = *data;
		test_struct.ps_cali_nvram_data[0] = *data & 0xff;
		test_struct.ps_cali_nvram_data[1] = (*data & 0xff00) >> 8;
    FLPLOGD("--------write----------\n");
	//FLPLOGD("--@perso1[0]:%d\n",test_struct.trace_nvram_data.info_name_perso1[0]);
	FLPLOGD("--@perso1[0]:%d\n",test_struct.ps_cali_nvram_data[0]);
	FLPLOGD("--@------------------\n");
    result = write(test_nvram_id.iFileDesc,&test_struct,rec_num*rec_size);
    if(result != rec_num*rec_size){
        FLPLOGE("write file failed\n");
        return -1;
    }
    if(!NVM_CloseFileDesc(test_nvram_id))
    {
        FLPLOGE("NVM_CloseFileDesc failed\n");
		return -1;
    }
    return 0;
}

static int ps_cali_init_priv(struct lps_priv *lps)
{
    memset(lps, 0x00, sizeof(*lps));
    lps->fd = -1;
    lps->dev = "/dev/als_ps";
    return 0;
}

//Calibration data file
static const char* ps_cali_file="/data/data/com.ckt.pscali_data";

/*---------------------------------------------------------------------------*/
static int alsps_open(struct lps_priv *lps)
{
    int err, max_retry = 3, retry_period = 100, retry;
    unsigned int flags = 1;
    if (lps->fd == -1) {
        lps->fd = open("/dev/als_ps", O_RDONLY);
        if (lps->fd < 0) {
            FLPLOGE("Couldn't open '%s' (%s)", lps->dev, strerror(errno));
            return -1;
        }
        retry = 0;
        while ((err = ioctl(lps->fd, ALSPS_SET_PS_MODE, &flags)) && (retry ++ < max_retry))
            usleep(retry_period*1000);
        if (err) {
            FLPLOGE("enable ps fail: %s", strerror(errno));
            return -1;
        }
        retry = 0;
        while ((err = ioctl(lps->fd, ALSPS_SET_ALS_MODE, &flags)) && (retry ++ < max_retry))
            usleep(retry_period*1000);
        if (err) {
            FLPLOGE("enable als fail: %s", strerror(errno));
            return -1;
        }
    }
    FLPLOGD("%s() %d\n", __func__, lps->fd);
    return 0;
}
/*---------------------------------------------------------------------------*/
static int alsps_close(struct lps_priv *lps)
{
    unsigned int flags = 0;
    int err;
    if (lps->fd != -1) {
        if ((err = ioctl(lps->fd, ALSPS_SET_PS_MODE, &flags))) {
            FLPLOGE("disable ps fail: %s", strerror(errno));
            return -1;
        } else if ((err = ioctl(lps->fd, ALSPS_SET_ALS_MODE, &flags))) {
            FLPLOGE("disable als fail: %s", strerror(errno));
            return -1;
        }
        close(lps->fd);
    }
    memset(lps, 0x00, sizeof(*lps));
    lps->fd = -1;
    lps->dev = "/dev/als_ps";
    return 0;
}
// xiangfei.peng 20130513
extern int phil_add_alsps_cali_in_entry(struct lps_priv *lps)
{
	int err = -EINVAL;
	unsigned int offset_data = 0;
	read_offset(&offset_data);
	FLPLOGD("[phil] cali in entry get ps_offset: %d\n", offset_data);
	if(lps->fd < 0)
	{
		FLPLOGE("[phil] alsps open failed!\n");
		return err;
	}
	if((err = ioctl(lps->fd, ALSPS_SET_PS_CALI, &offset_data))) {
		FLPLOGE("set ps_offset: %d(%s)\n", errno, strerror(errno));
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int alsps_update_info(struct lps_priv *lps)
{
    int err = -EINVAL;
    unsigned int als_dat, ps_dat;
	
    unsigned int offset_data=0;
    unsigned int ps_offset;
   if (!bUpToDate)
   {
    read_offset(&offset_data);
    ps_offset = offset_data;
     FLPLOGE("alsps_update_info set ps_offset: %d\n", ps_offset);
    if ((err = ioctl(lps->fd, ALSPS_SET_PS_CALI, &ps_offset))) {
        FLPLOGE("set ps_offset: %d(%s)\n", errno, strerror(errno));
    }
    else
	{
	 bUpToDate = true;
	}
   }
   
    if (lps->fd == -1) {
        FLPLOGE("invalid fd\n");
        return -EINVAL;
    } else if ((err = ioctl(lps->fd, ALSPS_GET_PS_RAW_DATA, &ps_dat))) {
        FLPLOGE("read ps  raw: %d(%s)\n", errno, strerror(errno));
        return err;
    } else if ((err = ioctl(lps->fd, ALSPS_GET_ALS_RAW_DATA, &als_dat))) {
        FLPLOGE("read als raw: %d(%s)\n", errno, strerror(errno));
        return err;
    }
    lps->als_raw = als_dat;
    lps->ps_raw = ps_dat;
    return 0;
}

/*---------------------------------------------------------------------------*/

/*
*   Description : Clear the calibration value in the file (set to -1)
*   Para:
*	@value: threshold
*   Return Value: 0 if no error
*		  nozero if error
*/
static int set_ps_cali(unsigned int value){
	FILE* fp=fopen(ps_cali_file,"w+");

	if(NULL==fp){
		FLPLOGD(TAG "Open calibration file failed.\n");
		return 1;
	}

	fprintf(fp, "%d",value);
	fclose(fp);

	return 0;
}

/*
*   Description : Calculate the threshold and store it into the file
*   Para:
*
*   Return Value: greater than 0 if no error
*		  Equal/less than 0 if error
*/
static unsigned int do_ps_cali(struct lps_priv *lps){

	unsigned int offset_data=0;
	int err=0;
	int i=0;

	if(lps->fd<0){
		FLPLOGD(TAG "wrong file descriptor.\n");
		return 0;
	}

	if(err = ioctl(lps->fd, ALSPS_GET_PS_CALI, &offset_data))
	{
		FLPLOGE("read ps raw: %d(%s)\n", errno, strerror(errno));

			return err;


	}


	return offset_data;

	}

static void  reset_ps_cali(struct lps_priv *lps){
	unsigned int flags;
	int err=0;
	int i=0;
	if(lps->fd<0){
		FLPLOGD(TAG "wrong file descriptor.\n");
		return 0;
	}
	flags = 1;
	if(err = ioctl(lps->fd, ALSPS_RESET_PS, &flags))
	{
		FLPLOGE("read als raw: %d(%s)\n", errno, strerror(errno));
        return err;
	}
	//prox_threshold_hi=((prox_max-prox_mean)*200+50)/100+prox_mean;
	//prox_threshold_lo=((prox_max-prox_mean)*170+50)/100+prox_mean;
    //	FLPLOGD(TAG "prox_max:%d, prox_mean:%d,prox_threshold_hi=%d,prox_threshold_lo=%d.\n",prox_max,prox_mean,prox_threshold_hi,prox_threshold_lo);

	//Only prox_threshold_hi used
	//if(set_ps_cali(offset_data)){
	//	return 0;
	//}
	//return prox_threshold_hi;
}

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void *ps_update_iv_thread(void *priv)
{
    struct lps_data *dat = (struct lps_data *)priv;
    struct lps_priv *lps = &dat->lps;
    struct itemview *iv = dat->iv;
    int err = 0, len = 0;
    char *status;
    static int op = -1;
	unsigned int cleardata = 0;
    FLPLOGD(TAG "%s: Start\n", __FUNCTION__);
    if ((err = alsps_open(lps))) {
		memset(dat->info, 0x00, sizeof(dat->info));
        sprintf(dat->info, "INIT FAILED\n");
        iv->redraw(iv);
        FLPLOGE("alsps() err = %d(%s)\n", err, dat->info);
        pthread_exit(NULL);
        return NULL;
    }

    while (1) {
        if (dat->exit_thd)
            break;

	pthread_mutex_lock(&dat->lps.evtmutex);
	if(op != dat->lps.pending_op)
	{
		op = dat->lps.pending_op;
		FLPLOGD("op: %d\n", dat->lps.pending_op);
	}
	pthread_mutex_unlock(&dat->lps.evtmutex);

	err = 0;
	if(op == PS_OP_CLEAR){
	//TODO     Clear Part
		if(read_write(&cleardata)==0){
			//snprintf(dat->lps.status, sizeof(dat->lps.status), "清除成功!\n");
			snprintf(dat->lps.status, sizeof(dat->lps.status), "clear successful!\n");
		}

             bUpToDate = false; 
		pthread_mutex_lock(&dat->lps.evtmutex);
		dat->lps.pending_op = PS_OP_NONE;
		pthread_mutex_unlock(&dat->lps.evtmutex);
	}
	else if(op == PS_OP_CALI_PRE){
		//snprintf(dat->lps.status, sizeof(dat->lps.status), "执行校准中，请勿触碰!\n");
		snprintf(dat->lps.status, sizeof(dat->lps.status), "cali ongoing,don't touch!\n");
		pthread_mutex_lock(&dat->lps.evtmutex);
		dat->lps.pending_op = PS_OP_CALI;
		dat->lps.cali_delay= 50; //50ms
		dat->lps.cali_num = 20;
		pthread_mutex_unlock(&dat->lps.evtmutex);
	}
	else if(op == PS_OP_CALI){
		//TODO Calibration Part
		unsigned int value=do_ps_cali(&(dat->lps));

		if(0 == read_write(&value)){
			//len = snprintf(dat->lps.status, sizeof(dat->lps.status), "校准: 成功! 偏移寄存器设置为:%d\n",value);
			len = snprintf(dat->lps.status, sizeof(dat->lps.status), "cali successful! the offset_data is:%d\n",value);
			dat->mod->test_result = FTM_TEST_PASS;
		}
		else{
			//len = snprintf(dat->lps.status, sizeof(dat->lps.status), "校准: 失败!\n");
			len = snprintf(dat->lps.status, sizeof(dat->lps.status), "cali: fail!\n");
			dat->mod->test_result = FTM_TEST_FAIL;
		}
              bUpToDate = false;
		pthread_mutex_lock(&dat->lps.evtmutex);
		dat->lps.pending_op = PS_OP_NONE;
		pthread_mutex_unlock(&dat->lps.evtmutex);
	}


        if ((err = alsps_update_info(lps)))
            continue;

        len = 0;
	//Remove ALS part
        //len += snprintf(dat->info+len, sizeof(dat->info)-len, "ALS: %4Xh (0:dark; +:bright)\n", lps->als_raw);
        len += snprintf(dat->info+len, sizeof(dat->info)-len, "distance: %4d(0: faraway; +: close)\n", lps->ps_raw);
	len += snprintf(dat->info+len, sizeof(dat->info)-len, "%s", lps->status);

        iv->set_text(iv, &dat->text);
        iv->redraw(iv);
    }

    alsps_close(lps);
    FLPLOGD(TAG "%s: Exit\n", __FUNCTION__);
    pthread_exit(NULL);
    return NULL;
}
/*---------------------------------------------------------------------------*/
int ps_cali_entry(struct ftm_param *param, void *priv)
{
    char *ptr;
    int chosen;
    bool exit = false;
    struct lps_data *dat = (struct lps_data *)priv;
    struct textview *tv;
    struct itemview *iv;
    struct statfs stat;
    int err,op;

    FLPLOGD(TAG "%s\n", __FUNCTION__);

    init_text(&dat->title, param->name, COLOR_YELLOW);
    init_text(&dat->text, &dat->info[0], COLOR_YELLOW);
    init_text(&dat->left_btn, "Fail", COLOR_YELLOW);
    init_text(&dat->center_btn, "Pass", COLOR_YELLOW);
    init_text(&dat->right_btn, "Back", COLOR_YELLOW);

    snprintf(dat->info, sizeof(dat->info), "初始化...\n");
    dat->exit_thd = false;
    bUpToDate = false; 

    if (!dat->iv) {
        iv = ui_new_itemview();
        if (!iv) {
            FLPLOGD(TAG "No memory");
            return -1;
        }
        dat->iv = iv;
    }
    iv = dat->iv;
    iv->set_title(iv, &dat->title);
    iv->set_items(iv, ps_cali_items, 0);
    iv->set_text(iv, &dat->text);

    pthread_create(&dat->update_thd, NULL, ps_update_iv_thread, priv);
    do {
        chosen = iv->run(iv, &exit);

        pthread_mutex_lock(&dat->lps.evtmutex);
        op = dat->lps.pending_op;
        pthread_mutex_unlock(&dat->lps.evtmutex);
        if ((chosen != ITEM_EXIT) && (op != PS_OP_NONE))   /*some OP is pending*/
            continue;

        switch (chosen) {
        case ITEM_CLEAR:
            pthread_mutex_lock(&dat->lps.evtmutex);
            dat->lps.pending_op = PS_OP_CLEAR;
            FLPLOGD("chosen clear: %d\n", dat->lps.pending_op);
            pthread_mutex_unlock(&dat->lps.evtmutex);
	    break;
        case ITEM_DOCALI:
            pthread_mutex_lock(&dat->lps.evtmutex);
            dat->lps.pending_op = PS_OP_CALI_PRE;
            dat->lps.cali_delay = 50;  //50ms
            dat->lps.cali_num   = 20;  //Use 20 samples
            FLPLOGD("chosen DOCALI\n");
            pthread_mutex_unlock(&dat->lps.evtmutex);
	    break;
        case ITEM_EXIT:
            exit = true;
            break;
        }

        if (exit) {
            dat->exit_thd = true;
            break;
        }
    } while (1);
    pthread_join(dat->update_thd, NULL);

    return 0;
}
/*---------------------------------------------------------------------------*/
int pscali_init(void)
{
    int ret = 0;
    struct ftm_module *mod;
    struct lps_data *dat;

    FLPLOGD(TAG "%s\n", __FUNCTION__);

    //mod = ftm_alloc(ITEM_PSCALI, sizeof(struct lps_data));
    mod = ftm_alloc(ITEM_PSCALI, sizeof(struct lps_data));
    dat  = mod_to_lps_data(mod);

    memset(dat, 0x00, sizeof(*dat));
    ps_cali_init_priv(&dat->lps);

    /*NOTE: the assignment MUST be done, or exception happens when tester press Test Pass/Test Fail*/
    dat->mod = mod;

    if (!mod)
        return -ENOMEM;

    ret = ftm_register(mod, ps_cali_entry, (void*)dat);

    return ret;
}
#endif

