/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   gc0311yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.2.3
 *
 * Author:
 * -------
 *   lanking zhou
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc0311yuv_Sensor.h"
#include "gc0311yuv_Camera_Sensor_para.h"
#include "gc0311yuv_CameraCustomized.h"

#define GC0311YUV_DEBUG
#ifdef GC0311YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

// ckt LiuHuojun 2012.11.28 16:06  ATV使用camera架构
//add by bonnie
#if defined(DTV_NMI5625) || defined(ATV_NMI168H)
extern bool g_bIsAtvStart;
#endif
//end

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 GC0311_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 2, GC0311_WRITE_ID);

}
kal_uint16 GC0311_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC0311_WRITE_ID);
	
    return get_byte;
}


/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   GC0311_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 GC0311_dummy_pixels = 0, GC0311_dummy_lines = 0;
kal_bool   GC0311_MODE_CAPTURE = KAL_FALSE;
kal_bool   GC0311_NIGHT_MODE = KAL_FALSE;

kal_uint32 GC0311_isp_master_clock;
static kal_uint32 GC0311_g_fPV_PCLK = 26;

kal_uint8 GC0311_sensor_write_I2C_address = GC0311_WRITE_ID;
kal_uint8 GC0311_sensor_read_I2C_address = GC0311_READ_ID;

UINT8 GC0311PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT GC0311SensorConfigData;

#define GC0311_SET_PAGE0 	GC0311_write_cmos_sensor(0xfe, 0x00)
#define GC0311_SET_PAGE1 	GC0311_write_cmos_sensor(0xfe, 0x01)

//ckt fengyongfei add 20130710
kal_bool GC0311_awbMode = AWB_MODE_AUTO;

void GC0311_GetExifInfo(UINT32 exifAddr)
{
     SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
     pExifInfo->AWBMode = GC0311_awbMode;
}
//end
/*************************************************************************
 * FUNCTION
 *	GC0311_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of GC0311 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0311_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_GC0311_Shutter */


/*************************************************************************
 * FUNCTION
 *	GC0311_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of GC0311 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 GC0311_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = GC0311_read_cmos_sensor(0x04);
	temp_reg2 = GC0311_read_cmos_sensor(0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* GC0311_read_shutter */


/*************************************************************************
 * FUNCTION
 *	GC0311_write_reg
 *
 * DESCRIPTION
 *	This function set the register of GC0311.
 *
 * PARAMETERS
 *	addr : the register index of GC0311
 *  para : setting parameter of the specified register of GC0311
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0311_write_reg(kal_uint32 addr, kal_uint32 para)
{
	GC0311_write_cmos_sensor(addr, para);
} /* GC0311_write_reg() */


/*************************************************************************
 * FUNCTION
 *	GC0311_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from GC0311.
 *
 * PARAMETERS
 *	addr : the register index of GC0311
 *
 * RETURNS
 *	the data that read from GC0311
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 GC0311_read_reg(kal_uint32 addr)
{
	return GC0311_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	GC0311_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void GC0311_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = GC0311_read_cmos_sensor(0x42);
	
	if (enalbe)
	{
		GC0311_write_cmos_sensor(0x42, (temp_AWB_reg |0x02));
	}
	else
	{
		GC0311_write_cmos_sensor(0x42, (temp_AWB_reg & (~0x02)));
	}

}


/*************************************************************************
* FUNCTION
*	GC0311_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC0311GammaSelect(kal_uint32 GammaLvl)
{
	switch(GammaLvl)
	{
		case GC0311_RGB_Gamma_m1:						//smallest gamma curve
			GC0311_write_cmos_sensor(0xfe, 0x00);
			GC0311_write_cmos_sensor(0xbf, 0x06);
			GC0311_write_cmos_sensor(0xc0, 0x12);
			GC0311_write_cmos_sensor(0xc1, 0x22);
			GC0311_write_cmos_sensor(0xc2, 0x35);
			GC0311_write_cmos_sensor(0xc3, 0x4b);
			GC0311_write_cmos_sensor(0xc4, 0x5f);
			GC0311_write_cmos_sensor(0xc5, 0x72);
			GC0311_write_cmos_sensor(0xc6, 0x8d);
			GC0311_write_cmos_sensor(0xc7, 0xa4);
			GC0311_write_cmos_sensor(0xc8, 0xb8);
			GC0311_write_cmos_sensor(0xc9, 0xc8);
			GC0311_write_cmos_sensor(0xca, 0xd4);
			GC0311_write_cmos_sensor(0xcb, 0xde);
			GC0311_write_cmos_sensor(0xcc, 0xe6);
			GC0311_write_cmos_sensor(0xcd, 0xf1);
			GC0311_write_cmos_sensor(0xce, 0xf8);
			GC0311_write_cmos_sensor(0xcf, 0xfd);
			break;
		case GC0311_RGB_Gamma_m2:
			GC0311_write_cmos_sensor(0xBF, 0x08);
			GC0311_write_cmos_sensor(0xc0, 0x0F);
			GC0311_write_cmos_sensor(0xc1, 0x21);
			GC0311_write_cmos_sensor(0xc2, 0x32);
			GC0311_write_cmos_sensor(0xc3, 0x43);
			GC0311_write_cmos_sensor(0xc4, 0x50);
			GC0311_write_cmos_sensor(0xc5, 0x5E);
			GC0311_write_cmos_sensor(0xc6, 0x78);
			GC0311_write_cmos_sensor(0xc7, 0x90);
			GC0311_write_cmos_sensor(0xc8, 0xA6);
			GC0311_write_cmos_sensor(0xc9, 0xB9);
			GC0311_write_cmos_sensor(0xcA, 0xC9);
			GC0311_write_cmos_sensor(0xcB, 0xD6);
			GC0311_write_cmos_sensor(0xcC, 0xE0);
			GC0311_write_cmos_sensor(0xcD, 0xEE);
			GC0311_write_cmos_sensor(0xcE, 0xF8);
			GC0311_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case GC0311_RGB_Gamma_m3:			
			GC0311_write_cmos_sensor(0xBF, 0x0B);
			GC0311_write_cmos_sensor(0xc0, 0x16);
			GC0311_write_cmos_sensor(0xc1, 0x29);
			GC0311_write_cmos_sensor(0xc2, 0x3C);
			GC0311_write_cmos_sensor(0xc3, 0x4F);
			GC0311_write_cmos_sensor(0xc4, 0x5F);
			GC0311_write_cmos_sensor(0xc5, 0x6F);
			GC0311_write_cmos_sensor(0xc6, 0x8A);
			GC0311_write_cmos_sensor(0xc7, 0x9F);
			GC0311_write_cmos_sensor(0xc8, 0xB4);
			GC0311_write_cmos_sensor(0xc9, 0xC6);
			GC0311_write_cmos_sensor(0xcA, 0xD3);
			GC0311_write_cmos_sensor(0xcB, 0xDD);
			GC0311_write_cmos_sensor(0xcC, 0xE5);
			GC0311_write_cmos_sensor(0xcD, 0xF1);
			GC0311_write_cmos_sensor(0xcE, 0xFA);
			GC0311_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case GC0311_RGB_Gamma_m4:
			GC0311_write_cmos_sensor(0xBF, 0x0E);
			GC0311_write_cmos_sensor(0xc0, 0x1C);
			GC0311_write_cmos_sensor(0xc1, 0x34);
			GC0311_write_cmos_sensor(0xc2, 0x48);
			GC0311_write_cmos_sensor(0xc3, 0x5A);
			GC0311_write_cmos_sensor(0xc4, 0x6B);
			GC0311_write_cmos_sensor(0xc5, 0x7B);
			GC0311_write_cmos_sensor(0xc6, 0x95);
			GC0311_write_cmos_sensor(0xc7, 0xAB);
			GC0311_write_cmos_sensor(0xc8, 0xBF);
			GC0311_write_cmos_sensor(0xc9, 0xCE);
			GC0311_write_cmos_sensor(0xcA, 0xD9);
			GC0311_write_cmos_sensor(0xcB, 0xE4);
			GC0311_write_cmos_sensor(0xcC, 0xEC);
			GC0311_write_cmos_sensor(0xcD, 0xF7);
			GC0311_write_cmos_sensor(0xcE, 0xFD);
			GC0311_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case GC0311_RGB_Gamma_m5:
			GC0311_write_cmos_sensor(0xBF, 0x10);
			GC0311_write_cmos_sensor(0xc0, 0x20);
			GC0311_write_cmos_sensor(0xc1, 0x38);
			GC0311_write_cmos_sensor(0xc2, 0x4E);
			GC0311_write_cmos_sensor(0xc3, 0x63);
			GC0311_write_cmos_sensor(0xc4, 0x76);
			GC0311_write_cmos_sensor(0xc5, 0x87);
			GC0311_write_cmos_sensor(0xc6, 0xA2);
			GC0311_write_cmos_sensor(0xc7, 0xB8);
			GC0311_write_cmos_sensor(0xc8, 0xCA);
			GC0311_write_cmos_sensor(0xc9, 0xD8);
			GC0311_write_cmos_sensor(0xcA, 0xE3);
			GC0311_write_cmos_sensor(0xcB, 0xEB);
			GC0311_write_cmos_sensor(0xcC, 0xF0);
			GC0311_write_cmos_sensor(0xcD, 0xF8);
			GC0311_write_cmos_sensor(0xcE, 0xFD);
			GC0311_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case GC0311_RGB_Gamma_m6:										// largest gamma curve
			GC0311_write_cmos_sensor(0xBF, 0x14);
			GC0311_write_cmos_sensor(0xc0, 0x28);
			GC0311_write_cmos_sensor(0xc1, 0x44);
			GC0311_write_cmos_sensor(0xc2, 0x5D);
			GC0311_write_cmos_sensor(0xc3, 0x72);
			GC0311_write_cmos_sensor(0xc4, 0x86);
			GC0311_write_cmos_sensor(0xc5, 0x95);
			GC0311_write_cmos_sensor(0xc6, 0xB1);
			GC0311_write_cmos_sensor(0xc7, 0xC6);
			GC0311_write_cmos_sensor(0xc8, 0xD5);
			GC0311_write_cmos_sensor(0xc9, 0xE1);
			GC0311_write_cmos_sensor(0xcA, 0xEA);
			GC0311_write_cmos_sensor(0xcB, 0xF1);
			GC0311_write_cmos_sensor(0xcC, 0xF5);
			GC0311_write_cmos_sensor(0xcD, 0xFB);
			GC0311_write_cmos_sensor(0xcE, 0xFE);
			GC0311_write_cmos_sensor(0xcF, 0xFF);
			break;
		case GC0311_RGB_Gamma_night:									//Gamma for night mode
			GC0311_write_cmos_sensor(0xBF, 0x0B);
			GC0311_write_cmos_sensor(0xc0, 0x16);
			GC0311_write_cmos_sensor(0xc1, 0x29);
			GC0311_write_cmos_sensor(0xc2, 0x3C);
			GC0311_write_cmos_sensor(0xc3, 0x4F);
			GC0311_write_cmos_sensor(0xc4, 0x5F);
			GC0311_write_cmos_sensor(0xc5, 0x6F);
			GC0311_write_cmos_sensor(0xc6, 0x8A);
			GC0311_write_cmos_sensor(0xc7, 0x9F);
			GC0311_write_cmos_sensor(0xc8, 0xB4);
			GC0311_write_cmos_sensor(0xc9, 0xC6);
			GC0311_write_cmos_sensor(0xcA, 0xD3);
			GC0311_write_cmos_sensor(0xcB, 0xDD);
			GC0311_write_cmos_sensor(0xcC, 0xE5);
			GC0311_write_cmos_sensor(0xcD, 0xF1);
			GC0311_write_cmos_sensor(0xcE, 0xFA);
			GC0311_write_cmos_sensor(0xcF, 0xFF);
			break;
		default:
			//GC0311_RGB_Gamma_m1
			GC0311_write_cmos_sensor(0xfe, 0x00);
			GC0311_write_cmos_sensor(0xbf, 0x06);
			GC0311_write_cmos_sensor(0xc0, 0x12);
			GC0311_write_cmos_sensor(0xc1, 0x22);
			GC0311_write_cmos_sensor(0xc2, 0x35);
			GC0311_write_cmos_sensor(0xc3, 0x4b);
			GC0311_write_cmos_sensor(0xc4, 0x5f);
			GC0311_write_cmos_sensor(0xc5, 0x72);
			GC0311_write_cmos_sensor(0xc6, 0x8d);
			GC0311_write_cmos_sensor(0xc7, 0xa4);
			GC0311_write_cmos_sensor(0xc8, 0xb8);
			GC0311_write_cmos_sensor(0xc9, 0xc8);
			GC0311_write_cmos_sensor(0xca, 0xd4);
			GC0311_write_cmos_sensor(0xcb, 0xde);
			GC0311_write_cmos_sensor(0xcc, 0xe6);
			GC0311_write_cmos_sensor(0xcd, 0xf1);
			GC0311_write_cmos_sensor(0xce, 0xf8);
			GC0311_write_cmos_sensor(0xcf, 0xfd);
			break;
	}
}


/*************************************************************************
 * FUNCTION
 *	GC0311_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of GC0311 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from GC0311
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0311_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* GC0311_config_window */


/*************************************************************************
 * FUNCTION
 *	GC0311_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 GC0311_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	GC0311_NightMode
 *
 * DESCRIPTION
 *	This function night mode of GC0311.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void GC0311NightMode(kal_bool bEnable)
{
	if (bEnable)
	{	
			GC0311_write_cmos_sensor(0xfe, 0x01);
		if(GC0311_MPEG4_encode_mode == KAL_TRUE){
			GC0311_write_cmos_sensor(0x2b, 0x04); 
			GC0311_write_cmos_sensor(0x2c, 0x80); 
			GC0311_write_cmos_sensor(0x33, 0x00);
		}
		else{
			GC0311_write_cmos_sensor(0x2b, 0x03); 
			GC0311_write_cmos_sensor(0x2c, 0x00); 
			GC0311_write_cmos_sensor(0x33, 0x30);
		}
             		GC0311_write_cmos_sensor(0xfe, 0x00);
			GC0311GammaSelect(GC0311_RGB_Gamma_night);		
			GC0311_NIGHT_MODE = KAL_TRUE;
	}
	else 
	{
			GC0311_write_cmos_sensor(0xfe, 0x01);
			GC0311_write_cmos_sensor(0x2b, 0x03); 
			GC0311_write_cmos_sensor(0x2c, 0x00); 
		if(GC0311_MPEG4_encode_mode == KAL_TRUE)
			GC0311_write_cmos_sensor(0x33, 0x00);
		else
			GC0311_write_cmos_sensor(0x33, 0x20);
           	       GC0311_write_cmos_sensor(0xfe, 0x00);
			GC0311GammaSelect(GC0311_RGB_Gamma_m4);				   
			GC0311_NIGHT_MODE = KAL_FALSE;
	}
} /* GC0311_NightMode */

/*************************************************************************
* FUNCTION
*	GC0311_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void GC0311_Sensor_Init(void)
{
	GC0311_write_cmos_sensor(0xfe, 0xf0);
	GC0311_write_cmos_sensor(0xfe, 0xf0);
	GC0311_write_cmos_sensor(0xfe, 0xf0);
	GC0311_write_cmos_sensor(0x42, 0x00);
	GC0311_write_cmos_sensor(0x4f, 0x00);
	GC0311_write_cmos_sensor(0x03, 0x02);
	GC0311_write_cmos_sensor(0x04, 0x88);
	GC0311_write_cmos_sensor(0xfc, 0x16);

	///////////////////////////////////////////////
	/////////// system reg ////////////////////////
	///////////////////////////////////////////////
	GC0311_write_cmos_sensor(0xf1, 0x07);
	GC0311_write_cmos_sensor(0xf2, 0x01);
	GC0311_write_cmos_sensor(0xfc, 0x16);
	///////////////////////////////////////////////
	/////////// CISCTL////////////////////////
	///////////////////////////////////////////////
	GC0311_write_cmos_sensor(0xfe, 0x00);
	//////window setting/////
	GC0311_write_cmos_sensor(0x0d, 0x01);
	GC0311_write_cmos_sensor(0x0e, 0xe8);
	GC0311_write_cmos_sensor(0x0f, 0x02);
	GC0311_write_cmos_sensor(0x10, 0x88);
	GC0311_write_cmos_sensor(0x09, 0x00);
	GC0311_write_cmos_sensor(0x0a, 0x00);
	GC0311_write_cmos_sensor(0x0b, 0x00);
	GC0311_write_cmos_sensor(0x0c, 0x04);
							
	GC0311_write_cmos_sensor(0x05, 0x01); 	
	GC0311_write_cmos_sensor(0x06, 0x32); 
	GC0311_write_cmos_sensor(0x07, 0x00);
	GC0311_write_cmos_sensor(0x08, 0x34);
	GC0311_SET_PAGE1;
	GC0311_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
	GC0311_write_cmos_sensor(0x2a, 0x32);   //anti-flicker step [7:0]

	GC0311_write_cmos_sensor(0x2b, 0x02);   //exp level 0  14.28fps
	GC0311_write_cmos_sensor(0x2c, 0x1c); 
	GC0311_write_cmos_sensor(0x2d, 0x02);   //exp level 1  12.50fps
	GC0311_write_cmos_sensor(0x2e, 0xd0); 
	GC0311_write_cmos_sensor(0x2f, 0x03);   //exp level 2  10.00fps
	GC0311_write_cmos_sensor(0x30, 0xc0); 
	GC0311_write_cmos_sensor(0x31, 0x04);   //exp level 3  5.00fps
	GC0311_write_cmos_sensor(0x32, 0xb0); 
	GC0311_write_cmos_sensor(0x33, 0x20);  
	GC0311_SET_PAGE0;
 /////////////////20120703//////////////////////////
	GC0311_write_cmos_sensor(0x77, 0x7c);
	GC0311_write_cmos_sensor(0x78, 0x40);
	GC0311_write_cmos_sensor(0x79, 0x56);
  /////////////////20120703//////////////////////////						  
	GC0311_write_cmos_sensor(0x17, 0x15);
	GC0311_write_cmos_sensor(0x19, 0x04);
	GC0311_write_cmos_sensor(0x1f, 0x08);
	GC0311_write_cmos_sensor(0x20, 0x01);
	GC0311_write_cmos_sensor(0x21, 0x48);
	GC0311_write_cmos_sensor(0x1b, 0x48);
	GC0311_write_cmos_sensor(0x22, 0xba);
	GC0311_write_cmos_sensor(0x23, 0x06);//  07--06 20120905
	GC0311_write_cmos_sensor(0x24, 0x16);
						   				   
	//global gain for range 	
	GC0311_write_cmos_sensor(0x70, 0x54);
	GC0311_write_cmos_sensor(0x73, 0x80);
	GC0311_write_cmos_sensor(0x76, 0x80);
	////////////////////////////////////////////////
	///////////////////////BLK//////////////////////
	////////////////////////////////////////////////
	GC0311_write_cmos_sensor(0x26, 0xf7);
	GC0311_write_cmos_sensor(0x28, 0x7f);
	GC0311_write_cmos_sensor(0x29, 0x40);
	GC0311_write_cmos_sensor(0x33, 0x18);
	GC0311_write_cmos_sensor(0x34, 0x18);
	GC0311_write_cmos_sensor(0x35, 0x18);
	GC0311_write_cmos_sensor(0x36, 0x18);

	////////////////////////////////////////////////
	//////////////block enable/////////////////////
	////////////////////////////////////////////////
	GC0311_write_cmos_sensor(0x40, 0xdf); 
	GC0311_write_cmos_sensor(0x41, 0x2e); 

	GC0311_write_cmos_sensor(0x42, 0xff);

	GC0311_write_cmos_sensor(0x44, 0xe0);//0xa0
	GC0311_write_cmos_sensor(0x46, 0x02);
	GC0311_write_cmos_sensor(0x4d, 0x01);
	GC0311_write_cmos_sensor(0x4f, 0x01);
	GC0311_write_cmos_sensor(0x7e, 0x08);
	GC0311_write_cmos_sensor(0x7f, 0xc3);
							
	//DN & EEINTP				
	GC0311_write_cmos_sensor(0x80, 0xe7);
	GC0311_write_cmos_sensor(0x82, 0x35);  /// 30
	GC0311_write_cmos_sensor(0x84, 0x02);
	GC0311_write_cmos_sensor(0x89, 0x22);
	GC0311_write_cmos_sensor(0x90, 0xbc);
	GC0311_write_cmos_sensor(0x92, 0x08);
	GC0311_write_cmos_sensor(0x94, 0x08);
	GC0311_write_cmos_sensor(0x95, 0x64);
							 
	/////////////////////ASDE/////////////
	GC0311_write_cmos_sensor(0x9a, 0x15);
	GC0311_write_cmos_sensor(0x9c, 0x46);

	///////////////////////////////////////
	////////////////Y gamma ///////////////////
	////////////////////////////////////////////
#if 1
	GC0311_write_cmos_sensor(0xfe, 0x00);
	GC0311_write_cmos_sensor(0x63, 0x00); 
	GC0311_write_cmos_sensor(0x64, 0x06); 
	GC0311_write_cmos_sensor(0x65, 0x0c); 
	GC0311_write_cmos_sensor(0x66, 0x18);
	GC0311_write_cmos_sensor(0x67, 0x2A);
	GC0311_write_cmos_sensor(0x68, 0x3D);
	GC0311_write_cmos_sensor(0x69, 0x50);
	GC0311_write_cmos_sensor(0x6A, 0x60);
	GC0311_write_cmos_sensor(0x6B, 0x80);
	GC0311_write_cmos_sensor(0x6C, 0xA0);
	GC0311_write_cmos_sensor(0x6D, 0xC0);
	GC0311_write_cmos_sensor(0x6E, 0xE0);
	GC0311_write_cmos_sensor(0x6F, 0xFF);
	GC0311_write_cmos_sensor(0xfe, 0x00);
#else	// 20130111
	GC0311_write_cmos_sensor(0xfe, 0x00);
	GC0311_write_cmos_sensor(0x63, 0x00); 
	GC0311_write_cmos_sensor(0x64, 0x05); 
	GC0311_write_cmos_sensor(0x65, 0x14); 
	GC0311_write_cmos_sensor(0x66, 0x21);
	GC0311_write_cmos_sensor(0x67, 0x48);
	GC0311_write_cmos_sensor(0x68, 0x59);
	GC0311_write_cmos_sensor(0x69, 0x6a);
	GC0311_write_cmos_sensor(0x6A, 0x7a);
	GC0311_write_cmos_sensor(0x6B, 0x97);
	GC0311_write_cmos_sensor(0x6C, 0xb1);
	GC0311_write_cmos_sensor(0x6D, 0xCb);
	GC0311_write_cmos_sensor(0x6E, 0xE6);
	GC0311_write_cmos_sensor(0x6F, 0xFF);
	GC0311_write_cmos_sensor(0xfe, 0x00);
#endif
	///////////////////////////////////////
	////////////////RGB gamma //////////////
	///////////////////////////////////////
	GC0311_write_cmos_sensor(0xBF, 0x0E);
	GC0311_write_cmos_sensor(0xc0, 0x1C);
	GC0311_write_cmos_sensor(0xc1, 0x34);
	GC0311_write_cmos_sensor(0xc2, 0x48);
	GC0311_write_cmos_sensor(0xc3, 0x5A);
	GC0311_write_cmos_sensor(0xc4, 0x6B);
	GC0311_write_cmos_sensor(0xc5, 0x7B);
	GC0311_write_cmos_sensor(0xc6, 0x95);
	GC0311_write_cmos_sensor(0xc7, 0xAB);
	GC0311_write_cmos_sensor(0xc8, 0xBF);
	GC0311_write_cmos_sensor(0xc9, 0xCE);
	GC0311_write_cmos_sensor(0xcA, 0xD9);
	GC0311_write_cmos_sensor(0xcB, 0xE4);
	GC0311_write_cmos_sensor(0xcC, 0xEC);
	GC0311_write_cmos_sensor(0xcD, 0xF7);
	GC0311_write_cmos_sensor(0xcE, 0xFD);
	GC0311_write_cmos_sensor(0xcF, 0xFF);

	////////////////////////////
	/////////////YCP//////////////
	////////////////////////////
	GC0311_write_cmos_sensor(0xfe, 0x00);
	GC0311_write_cmos_sensor(0xd1, 0x30);//0x36
	GC0311_write_cmos_sensor(0xd2, 0x30);//0x36
	GC0311_write_cmos_sensor(0xdd, 0x00);
	GC0311_write_cmos_sensor(0xed, 0x00);
	 
	GC0311_write_cmos_sensor(0xd3, 0x40);// 20130111 _ 0x32
	//GC0311_write_cmos_sensor(0xd6, 0xfa);// 20130111
	//GC0311_write_cmos_sensor(0xd7, 0x16);// 20130111
	//GC0311_write_cmos_sensor(0xd8, 0x18);// 20130111
	 
	GC0311_write_cmos_sensor(0xde, 0x38);

	GC0311_write_cmos_sensor(0xe4, 0x88);
	GC0311_write_cmos_sensor(0xe5, 0x40);

	GC0311_write_cmos_sensor(0xfe, 0x01);
	GC0311_write_cmos_sensor(0x18, 0x22);

	//////////////////////////////////
	///////////MEANSURE WINDOW////////
	/////////////////////////////////
	GC0311_write_cmos_sensor(0x08, 0xa4);
	GC0311_write_cmos_sensor(0x09, 0xf0);

	///////////////////////////////////////////////
	/////////////// AEC ////////////////////////
	///////////////////////////////////////////////
	GC0311_write_cmos_sensor(0xfe, 0x01);
	GC0311_write_cmos_sensor(0x10, 0x00);//08
	GC0311_write_cmos_sensor(0x11, 0x11);
	GC0311_write_cmos_sensor(0x12, 0x24);  ///  14
	GC0311_write_cmos_sensor(0x13, 0x78);//0x70
	GC0311_write_cmos_sensor(0x16, 0xd8);
	GC0311_write_cmos_sensor(0x17, 0x98);
	GC0311_write_cmos_sensor(0x18, 0x01);
	GC0311_write_cmos_sensor(0x21, 0xc0);
	GC0311_write_cmos_sensor(0x22, 0x50);  ///  40

	//////////////////////////////
	/////////////AWB///////////////
	////////////////////////////////
	GC0311_write_cmos_sensor(0x06, 0x10);
	GC0311_write_cmos_sensor(0x08, 0xa0);
	GC0311_write_cmos_sensor(0x50, 0xfe);
	GC0311_write_cmos_sensor(0x51, 0x05);
	GC0311_write_cmos_sensor(0x52, 0x08);
	GC0311_write_cmos_sensor(0x53, 0x05);
	GC0311_write_cmos_sensor(0x54, 0x10);  /// 10
	GC0311_write_cmos_sensor(0x55, 0x20);
	GC0311_write_cmos_sensor(0x56, 0x16);
	GC0311_write_cmos_sensor(0x57, 0x10);
	GC0311_write_cmos_sensor(0x58, 0xf0);
	GC0311_write_cmos_sensor(0x59, 0x10);
	GC0311_write_cmos_sensor(0x5a, 0x10);
	GC0311_write_cmos_sensor(0x5b, 0xf0);
	GC0311_write_cmos_sensor(0x5e, 0xe8);
	GC0311_write_cmos_sensor(0x5f, 0x20);
	GC0311_write_cmos_sensor(0x60, 0x20);
	GC0311_write_cmos_sensor(0x61, 0xe0);
							
	GC0311_write_cmos_sensor(0x62, 0x03);
	GC0311_write_cmos_sensor(0x63, 0x30);
	GC0311_write_cmos_sensor(0x64, 0xc0);
	GC0311_write_cmos_sensor(0x65, 0xd0);
	GC0311_write_cmos_sensor(0x66, 0x20);
	GC0311_write_cmos_sensor(0x67, 0x00);

	#if 1
        GC0311_write_cmos_sensor(0x6d, 0x40);
	GC0311_write_cmos_sensor(0x6e, 0x08);
	GC0311_write_cmos_sensor(0x6f, 0x08);
	GC0311_write_cmos_sensor(0x70, 0x10);
	GC0311_write_cmos_sensor(0x71, 0x62);
	GC0311_write_cmos_sensor(0x72, 0x2e);//26 fast mode
	GC0311_write_cmos_sensor(0x73, 0x72);
	GC0311_write_cmos_sensor(0x74, 0x23);

        #else
        GC0311_write_cmos_sensor(0x6d, 0x32);// 0x40	20130111
	GC0311_write_cmos_sensor(0x6e, 0x08);
	GC0311_write_cmos_sensor(0x6f, 0x08);
	GC0311_write_cmos_sensor(0x70, 0xb0);// 0x10	20130111
	GC0311_write_cmos_sensor(0x71, 0x62);
	GC0311_write_cmos_sensor(0x72, 0x26);//26 fast mode // 0x2e	20130111
	GC0311_write_cmos_sensor(0x73, 0x62);// 0x71	20130111
	GC0311_write_cmos_sensor(0x74, 0x13);// 0x23	20130111
	#endif						
	GC0311_write_cmos_sensor(0x75, 0x40);
	GC0311_write_cmos_sensor(0x76, 0x48);
	GC0311_write_cmos_sensor(0x77, 0xc2);
	GC0311_write_cmos_sensor(0x78, 0xa5);
							 
	GC0311_write_cmos_sensor(0x79, 0x18);
	GC0311_write_cmos_sensor(0x7a, 0x40);
	GC0311_write_cmos_sensor(0x7b, 0xb0);
	GC0311_write_cmos_sensor(0x7c, 0xf5);
							 
	GC0311_write_cmos_sensor(0x81, 0x70);//0x80
	GC0311_write_cmos_sensor(0x82, 0x50);//0x60
	GC0311_write_cmos_sensor(0x83, 0xa0);   /// a0
							
	GC0311_write_cmos_sensor(0x8a, 0xf8);
	GC0311_write_cmos_sensor(0x8b, 0xf4);
	GC0311_write_cmos_sensor(0x8c, 0x0a);
	GC0311_write_cmos_sensor(0x8d, 0x00);
	GC0311_write_cmos_sensor(0x8e, 0x00);
	GC0311_write_cmos_sensor(0x8f, 0x00);
	GC0311_write_cmos_sensor(0x90, 0x12);
							
	GC0311_write_cmos_sensor(0xfe, 0x00);

	///////////////////////////////////////////////
	/////////// SPI reciver////////////////////////
	///////////////////////////////////////////////
	GC0311_write_cmos_sensor(0xad, 0x00);

	/////////////////////////////
	///////////LSC///////////////
	/////////////////////////////
	//GC0311_write_cmos_sensor(0xfe, 0x01);// 20130111
	//GC0311_write_cmos_sensor(0x9c, 0x00);// 20130111
	//GC0311_write_cmos_sensor(0x9f, 0x40);// 20130111

	/////////////////////////////
	///////////LSC///////////////
	/////////////////////////////
	GC0311_write_cmos_sensor(0xfe, 0x01);
	GC0311_write_cmos_sensor(0xa0, 0x00);
	GC0311_write_cmos_sensor(0xa1, 0x3c);
	GC0311_write_cmos_sensor(0xa2, 0x50);
	GC0311_write_cmos_sensor(0xa3, 0x00);
	GC0311_write_cmos_sensor(0xa8, 0x09);
	GC0311_write_cmos_sensor(0xa9, 0x04);
	GC0311_write_cmos_sensor(0xaa, 0x00);
	GC0311_write_cmos_sensor(0xab, 0x0c);
	GC0311_write_cmos_sensor(0xac, 0x02);
	GC0311_write_cmos_sensor(0xad, 0x00);
	GC0311_write_cmos_sensor(0xae, 0x15);
	GC0311_write_cmos_sensor(0xaf, 0x05);
	GC0311_write_cmos_sensor(0xb0, 0x00);
	GC0311_write_cmos_sensor(0xb1, 0x0f);
	GC0311_write_cmos_sensor(0xb2, 0x06);
	GC0311_write_cmos_sensor(0xb3, 0x00);
	GC0311_write_cmos_sensor(0xb4, 0x36);
	GC0311_write_cmos_sensor(0xb5, 0x2a);
	GC0311_write_cmos_sensor(0xb6, 0x25);
	GC0311_write_cmos_sensor(0xba, 0x36);
	GC0311_write_cmos_sensor(0xbb, 0x25);
	GC0311_write_cmos_sensor(0xbc, 0x22);
	GC0311_write_cmos_sensor(0xc0, 0x1e);
	GC0311_write_cmos_sensor(0xc1, 0x18);
	GC0311_write_cmos_sensor(0xc2, 0x17);
	GC0311_write_cmos_sensor(0xc6, 0x1c);
	GC0311_write_cmos_sensor(0xc7, 0x18);
	GC0311_write_cmos_sensor(0xc8, 0x17);
	GC0311_write_cmos_sensor(0xb7, 0x00);
	GC0311_write_cmos_sensor(0xb8, 0x00);
	GC0311_write_cmos_sensor(0xb9, 0x00);
	GC0311_write_cmos_sensor(0xbd, 0x00);
	GC0311_write_cmos_sensor(0xbe, 0x00);
	GC0311_write_cmos_sensor(0xbf, 0x00);
	GC0311_write_cmos_sensor(0xc3, 0x00);
	GC0311_write_cmos_sensor(0xc4, 0x00);
	GC0311_write_cmos_sensor(0xc5, 0x00);
	GC0311_write_cmos_sensor(0xc9, 0x00);
	GC0311_write_cmos_sensor(0xca, 0x00);
	GC0311_write_cmos_sensor(0xcb, 0x00);
	GC0311_write_cmos_sensor(0xa4, 0x00);
	GC0311_write_cmos_sensor(0xa5, 0x00);
	GC0311_write_cmos_sensor(0xa6, 0x00);
	GC0311_write_cmos_sensor(0xa7, 0x00);

	GC0311_write_cmos_sensor(0xfe, 0x00);
	GC0311_write_cmos_sensor(0x50, 0x01);
	GC0311_write_cmos_sensor(0x44, 0xa2);//0xa0
	GC0311_SET_PAGE0;
	GC0311_write_cmos_sensor(0x17, 0x14);//mod by liutao 20130523 for camera upside down(b1:updown b0:mirror)

        GC0311_SET_PAGE0;
        GC0311_write_cmos_sensor(0x17, 0x14);//mod by liutao 20130523 for camera upside down(b1:updown b0:mirror)
}



UINT32 GC0311GetSensorID(UINT32 *sensorID)
{
    kal_uint16 sensor_id=0;
    int i;

    Sleep(20);

    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = GC0311_read_cmos_sensor(0xf0);
	            	printk("GC0311 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == GC0311_SENSOR_ID)             
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != GC0311_SENSOR_ID)
    {
        SENSORDB("GC0311 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    *sensorID = sensor_id;

    RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
	
    return ERROR_NONE;
}




/*************************************************************************
* FUNCTION
*	GC0311_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_GC0311() directly.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC0311_Write_More_Registers(void)
{

////////////////20120613 start//////////////////////////
	GC0311_write_cmos_sensor(0xfe, 0x01);
	GC0311_write_cmos_sensor(0x74, 0x03);
	GC0311_write_cmos_sensor(0x15, 0xfe);
	GC0311_write_cmos_sensor(0x21, 0xd8);
	
	GC0311_write_cmos_sensor(0xfe, 0x00);
	GC0311_write_cmos_sensor(0x41, 0x6e);
	GC0311_write_cmos_sensor(0x83, 0x03);
	GC0311_write_cmos_sensor(0x7e, 0x08);
	GC0311_write_cmos_sensor(0x9c, 0x64);
	GC0311_write_cmos_sensor(0x95, 0x54);//0x65
        GC0311_write_cmos_sensor(0xa5, 0x33);  ///  31
	
	GC0311_write_cmos_sensor(0xd1, 0x22);// 0x2d _ 0x22 0x34		20130111
	GC0311_write_cmos_sensor(0xd2, 0x22);// 0x2d _ 0x22 0x34		20130111

	///YUV maxtrix////
	GC0311_write_cmos_sensor(0xb0, 0x13);
    	GC0311_write_cmos_sensor(0xb1, 0x26);
	GC0311_write_cmos_sensor(0xb2, 0x07);
	GC0311_write_cmos_sensor(0xb3, 0xf5);
	GC0311_write_cmos_sensor(0xb4, 0xea);
	GC0311_write_cmos_sensor(0xb5, 0x22);  /// 21
	GC0311_write_cmos_sensor(0xb6, 0x20);   /// 21
	GC0311_write_cmos_sensor(0xb7, 0xe3);  /// e4
	GC0311_write_cmos_sensor(0xb8, 0xfb);

////////////////20120613 end//////////////////////////
    	GC0311GammaSelect(GC0311_RGB_Gamma_m4);//0:use default //GC0311_RGB_Gamma_m4
}


/*************************************************************************
 * FUNCTION
 *	GC0311Open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 GC0311Open(void)
{
    kal_uint16 sensor_id=0;
    int i;

	mt_set_gpio_mode(GPIO123,GPIO_MODE_01);
	mt_set_gpio_dir(GPIO123,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO141,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO141,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO142,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO142,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO143,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO143,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO144,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO144,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO145,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO145,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO146,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO146,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO153,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO153,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO154,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO154,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO155,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO155,GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO156,GPIO_MODE_02);
	mt_set_gpio_dir(GPIO156,GPIO_DIR_IN);
    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = GC0311_read_cmos_sensor(0xf0);
	            	printk("GC0311 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == GC0311_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != GC0311_SENSOR_ID)
    {
        SENSORDB("GC0311 Sensor id read failed, ID = %x\n", sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	
    GC0311_MPEG4_encode_mode = KAL_FALSE;
    RETAILMSG(1, (TEXT("Sensor Read ID OK \r\n")));
    // initail sequence write in
    GC0311_Sensor_Init();
    GC0311_Write_More_Registers();
	
    return ERROR_NONE;
} /* GC0311Open */


/*************************************************************************
 * FUNCTION
 *	GC0311Close
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 GC0311Close(void)
{
    return ERROR_NONE;
} /* GC0311Close */


/*************************************************************************
 * FUNCTION
 * GC0311Preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 GC0311Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1, (TEXT("Camera Video preview\r\n")));
        GC0311_MPEG4_encode_mode = KAL_TRUE;
       
    }
    else
    {
        RETAILMSG(1, (TEXT("Camera preview\r\n")));
        GC0311_MPEG4_encode_mode = KAL_FALSE;
    }
    Sleep(100);
    GC0311_write_cmos_sensor(0x26, 0x00);
    GC0311_write_cmos_sensor(0x2a, GC0311_read_cmos_sensor(0x2a));
    GC0311_write_cmos_sensor(0x2b, GC0311_read_cmos_sensor(0x2b));
    GC0311_write_cmos_sensor(0x2c, GC0311_read_cmos_sensor(0x2c));
    GC0311_write_cmos_sensor(0x2d, GC0311_read_cmos_sensor(0x2d));
    GC0311_write_cmos_sensor(0x2e, GC0311_read_cmos_sensor(0x2e));
    GC0311_write_cmos_sensor(0x2f, GC0311_read_cmos_sensor(0x2f));
    GC0311_write_cmos_sensor(0x30, GC0311_read_cmos_sensor(0x30));
    GC0311_write_cmos_sensor(0x31, GC0311_read_cmos_sensor(0x31));
    GC0311_write_cmos_sensor(0x26, 0xf7);
    GC0311_write_cmos_sensor(0x4f, 0x00);
    GC0311_write_cmos_sensor(0x03, GC0311_read_cmos_sensor(0x03));
    GC0311_write_cmos_sensor(0x04, GC0311_read_cmos_sensor(0x04));
    GC0311_write_cmos_sensor(0x4f, 0x01);

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&GC0311SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* GC0311Preview */


/*************************************************************************
 * FUNCTION
 *	GC0311Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 GC0311Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    GC0311_MODE_CAPTURE=KAL_TRUE;

    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&GC0311SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* GC0311_Capture() */



UINT32 GC0311GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight = IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* GC0311GetResolution() */


UINT32 GC0311GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

    /*pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;*/
    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 5;
    pSensorInfo->VideoDelayFrame = 4;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
    default:
        pSensorInfo->SensorClockFreq=12;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    }
    GC0311PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &GC0311SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* GC0311GetInfo() */


UINT32 GC0311Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
       // GC0311Capture(pImageWindow, pSensorConfigData);
	 GC0311Preview(pImageWindow, pSensorConfigData);
        break;
    }


    return TRUE;
}	/* GC0311Control() */

BOOL GC0311_set_param_wb(UINT16 para)
{

	GC0311_awbMode = para;//ckt fengyongfei add 20130710

	switch (para)
	{
		/*case AWB_MODE_OFF:
		//ckt fengyongfei add 20130710
			//GC0311_awb_enable(KAL_FALSE);
			//GC0311_write_cmos_sensor(0x42, 0x00);
		//end
		break;*/
		
		case AWB_MODE_AUTO:
			GC0311_awb_enable(KAL_TRUE);
//			GC0311_write_cmos_sensor(0x42, 0x00);//ckt fengyongfei add 20130710
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			GC0311_awb_enable(KAL_FALSE);
			GC0311_write_cmos_sensor(0x77, 0x8c); //WB_manual_gain 
			GC0311_write_cmos_sensor(0x78, 0x50);
			GC0311_write_cmos_sensor(0x79, 0x40);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny
			GC0311_awb_enable(KAL_FALSE);
			GC0311_write_cmos_sensor(0x77, 0x74); 
			GC0311_write_cmos_sensor(0x78, 0x52);
			GC0311_write_cmos_sensor(0x79, 0x40);			
		break;
		
		case AWB_MODE_INCANDESCENT: //office
			GC0311_awb_enable(KAL_FALSE);
			GC0311_write_cmos_sensor(0x77, 0x48);
			GC0311_write_cmos_sensor(0x78, 0x40);
			GC0311_write_cmos_sensor(0x79, 0x5c);
		break;
		
		case AWB_MODE_TUNGSTEN: //home
			GC0311_awb_enable(KAL_FALSE);
			GC0311_write_cmos_sensor(0x77, 0x40);
			GC0311_write_cmos_sensor(0x78, 0x54);
			GC0311_write_cmos_sensor(0x79, 0x70);
		break;
		
		case AWB_MODE_FLUORESCENT:
			GC0311_awb_enable(KAL_FALSE);
			GC0311_write_cmos_sensor(0x77, 0x40);
			GC0311_write_cmos_sensor(0x78, 0x42);
			GC0311_write_cmos_sensor(0x79, 0x50);
		break;
		
		default:
		return FALSE;
	}

	return TRUE;
} /* GC0311_set_param_wb */


BOOL GC0311_set_param_effect(UINT16 para)
{
	kal_uint32  ret = KAL_TRUE;

	switch (para)
	{
		case MEFFECT_OFF:
			GC0311_write_cmos_sensor(0x43 , 0x00);
		break;
		
		case MEFFECT_SEPIA:
			GC0311_write_cmos_sensor(0x43 , 0x02);
			GC0311_write_cmos_sensor(0xda , 0xd0);
			GC0311_write_cmos_sensor(0xdb , 0x28);
		break;
		
		case MEFFECT_NEGATIVE:
			GC0311_write_cmos_sensor(0x43 , 0x01);
		break;
		
		case MEFFECT_SEPIAGREEN:
			GC0311_write_cmos_sensor(0x43 , 0x02);
			GC0311_write_cmos_sensor(0xda , 0xc0);
			GC0311_write_cmos_sensor(0xdb , 0xc0);
		break;
		
		case MEFFECT_SEPIABLUE:
			GC0311_write_cmos_sensor(0x43 , 0x02);
			GC0311_write_cmos_sensor(0xda , 0x50);
			GC0311_write_cmos_sensor(0xdb , 0xe0);
		break;

		case MEFFECT_MONO:
			GC0311_write_cmos_sensor(0x43 , 0x02);
			GC0311_write_cmos_sensor(0xda , 0x00);
			GC0311_write_cmos_sensor(0xdb , 0x00);
		break;
		default:
			ret = FALSE;
	}

	return ret;

} /* GC0311_set_param_effect */


BOOL GC0311_set_param_banding(UINT16 para)
{
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			GC0311_write_cmos_sensor(0x05, 0x01); 	
			GC0311_write_cmos_sensor(0x06, 0x32); 
			GC0311_write_cmos_sensor(0x07, 0x00);
			GC0311_write_cmos_sensor(0x08, 0x34);
			
			GC0311_SET_PAGE1;
			GC0311_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			GC0311_write_cmos_sensor(0x2a, 0x3c);   //anti-flicker step [7:0]
			
			GC0311_write_cmos_sensor(0x2b, 0x02);   //exp level 0  14.28fps
			GC0311_write_cmos_sensor(0x2c, 0x1c); // 2a0
			GC0311_write_cmos_sensor(0x2d, 0x02);   //exp level 1  12.50fps
			GC0311_write_cmos_sensor(0x2e, 0xd0); // 300
			GC0311_write_cmos_sensor(0x2f, 0x03);   //exp level 2  10.00fps
			GC0311_write_cmos_sensor(0x30, 0xc0);  // 3c0
			GC0311_write_cmos_sensor(0x31, 0x04);   //exp level 3  7.14fps
			GC0311_write_cmos_sensor(0x32, 0xb0); // 540
			GC0311_SET_PAGE0;
			break;

		case AE_FLICKER_MODE_60HZ:
			GC0311_write_cmos_sensor(0x05, 0x01); 	
			GC0311_write_cmos_sensor(0x06, 0x32); 
			GC0311_write_cmos_sensor(0x07, 0x00);
			GC0311_write_cmos_sensor(0x08, 0x0c);
			
			GC0311_SET_PAGE1;
			GC0311_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			GC0311_write_cmos_sensor(0x2a, 0x32);   //anti-flicker step [7:0]
			
			GC0311_write_cmos_sensor(0x2b, 0x01);   //exp level 0  15.00fps
			GC0311_write_cmos_sensor(0x2c, 0xf4); // 270
			GC0311_write_cmos_sensor(0x2d, 0x02);   //exp level 0  12.00fps
			GC0311_write_cmos_sensor(0x2e, 0xbc); // 30c
			GC0311_write_cmos_sensor(0x2f, 0x03);   //exp level 0  10.00fps
			GC0311_write_cmos_sensor(0x30, 0xb6); // 3a8
			GC0311_write_cmos_sensor(0x31, 0x04);   //exp level 0  7.05fps
			GC0311_write_cmos_sensor(0x32, 0xb0); // 52e
			GC0311_SET_PAGE0;
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* GC0311_set_param_banding */


BOOL GC0311_set_param_exposure(UINT16 para)
{
/*
	kal_uint8 REG42;
	REG42 = GC0311_read_cmos_sensor(0x42);
	if(AE_EV_COMP_00 == para)
		REG42|=0x01;
	else
		REG42&=0xfe; 
*/

	switch (para)
	{
		case AE_EV_COMP_n13:
		case AE_EV_COMP_n10:
		case AE_EV_COMP_n07:
		case AE_EV_COMP_n03:

			GC0311_write_cmos_sensor(0xfe, 0x01);
			GC0311_write_cmos_sensor(0x10, 0x00);
			GC0311_write_cmos_sensor(0x13, 0x40);
			GC0311_write_cmos_sensor(0xfe, 0x00);
			break;
		case AE_EV_COMP_00:
			GC0311_write_cmos_sensor(0xfe, 0x01);
			GC0311_write_cmos_sensor(0x10, 0x00);
			GC0311_write_cmos_sensor(0x13, 0x78);
			GC0311_write_cmos_sensor(0xfe, 0x00);
		break;

		case AE_EV_COMP_03:
		case AE_EV_COMP_07:
		case AE_EV_COMP_10:
			
			GC0311_write_cmos_sensor(0xfe, 0x01);
			GC0311_write_cmos_sensor(0x10, 0x08);
			GC0311_write_cmos_sensor(0x13, 0x78);
			GC0311_write_cmos_sensor(0xfe, 0x00);
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* GC0311_set_param_exposure */



UINT32 GC0311YUVSetVideoMode(UINT16 u2FrameRate)    // lanking add
{
  
        GC0311_MPEG4_encode_mode = KAL_TRUE;
     if (u2FrameRate == 30)
   	{
   	
   	    /*********video frame ************/
		
   	}
    else if (u2FrameRate == 15)       
    	{
    	
   	    /*********video frame ************/
		
    	}
    else
   	{
   	
            SENSORDB("Wrong Frame Rate"); 
			
   	}

      return TRUE;

}


UINT32 GC0311YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    switch (iCmd) {
    case FID_AWB_MODE:
        GC0311_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        GC0311_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        GC0311_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        GC0311_set_param_banding(iPara);
		break;
	case FID_SCENE_MODE:
		GC0311NightMode(iPara);
        break;
    default:
        break;
    }
    return TRUE;
} /* GC0311YUVSensorSetting */


UINT32 GC0311FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 GC0311SensorRegNumber;
    UINT32 i;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    RETAILMSG(1, (_T("gaiyang GC0311FeatureControl FeatureId=%d\r\n"), FeatureId));


    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+GC0311_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+GC0311_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = GC0311_g_fPV_PCLK;
        *pFeatureParaLen=4; 
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        //GC0311NightMode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        GC0311_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        GC0311_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = GC0311_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &GC0311SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        GC0311YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:    //  lanking
	 GC0311YUVSetVideoMode(*pFeatureData16);
	 break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	GC0311GetSensorID(pFeatureData32);
	break;
//ckt fengyongfei add 20130710
    case SENSOR_FEATURE_GET_EXIF_INFO:
	GC0311_GetExifInfo(*pFeatureData32);
	break;
//end
    default:
        break;
	}
return ERROR_NONE;
}	/* GC0311FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncGC0311YUV=
{
	GC0311Open,
	GC0311GetInfo,
	GC0311GetResolution,
	GC0311FeatureControl,
	GC0311Control,
	GC0311Close
};


// ckt LiuHuojun 2012.11.28 16:06  ATV使用camera架构
//add by bonnie
#if defined(DTV_NMI5625) || defined(ATV_NMI168H)
extern UINT32 NMI60X_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc) ;
#endif

UINT32 GC0311_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{

// ckt LiuHuojun 2012.11.28 16:06  ATV使用camera架构
//add by bonnie
#if defined(DTV_NMI5625) || defined(ATV_NMI168H)
			if ( g_bIsAtvStart )
			{
					NMI60X_YUV_SensorInit(pfFunc);
					printk("GC0311_YUV_SensorInit---------------------NMI60X_YUV_SensorInit");
					return ERROR_NONE;
			}
#endif

	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC0311YUV;
	return ERROR_NONE;
} /* SensorInit() */
