/* Copyright Statement:
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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
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


#ifdef BUILD_LK
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM8019A_CS					0x8019

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static LCM_UTIL_FUNCS lcm_util = {0};

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg											lcm_util.dsi_read_reg()
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

{0x00, 1 , {0x00}},
{0xFF,  3 ,{0x80,0x19,0x01}},

{0x00, 1 , {0x80}},
{0xFF,  2 ,{0x80,0x19}}, 

{0x00, 1 , {0x8A}},
{0xC4,  1 ,{0x40}},

{0x00, 1 , {0xA6}},
{0xB3,  2 ,{0x20,0x01}},

{0x00, 1 , {0x90}},
{0xC0,  6 ,{0x00,0x15,0x00,0x00,0x00,0x03}},

{0x00, 1 , {0xB4}},
{0xC0,  1 ,{0x70}},

{0x00, 1 , {0x81}},
{0xC4,  1 ,{0x81}},

{0x00, 1 , {0x82}},
{0xC5,  1 ,{0xB0}},

{0x00, 1 , {0x90}},
{0xC5,  7 ,{0x4E,0x57,0x06,0x91,0x33,0x34,0x23}},
 
{0x00, 1 , {0xB1}},
{0xC5,  1 ,{0xA8}},

{0x00, 1 , {0x00}},
{0xD8,  2 ,{0x6F,0x6F}},

{0x00, 1 , {0x00}},
{0xD9,  1 ,{0x54}},

{0x00, 1 , {0x80}},
{0xCE,  12 ,{0x86,0x01,0x00,0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xA0}},
{0xCE,  14 ,{0x18,0x05,0x83,0x39,0x00,0x00,0x00,0x18,0x04,0x83,0x3A,0x00,0x00,0x00}},

{0x00, 1 , {0xB0}},
{0xCE,  14 ,{0x18,0x03,0x83,0x3B,0x86,0x00,0x00,0x18,0x02,0x83,0x3C,0x88,0x00,0x00}},

{0x00, 1 , {0xC0}},
{0xCF,  10 ,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x02,0x00,0x00}},

{0x00, 1 , {0xD0}},
{0xCF,  1 ,{0x00}},

{0x00, 1 , {0xC0}},
{0xCB,  15 ,{0x00,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xD0}},
{0xCB,  1 ,{0x00}},

{0x00, 1 , {0xD5}},
{0xCB,  10 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xE0}},
{0xCB,  6 ,{0x01,0x01,0x01,0x01,0x01,0x00}},

{0x00, 1 , {0x80}},
{0xCC,  10 ,{0x00,0x26,0x09,0x0B,0x01,0x25,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0x90}},
{0xCC,  6 ,{0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0x9A}},
{0xCC,  5 ,{0x00,0x00,0x00,0x00,0x00}},
 
{0x00, 1 , {0xA0}},
{0xCC,  11 ,{0x00,0x00,0x00,0x00,0x00,0x25,0x02,0x0c,0x0a,0x26,0x00}},

{0x00, 1 , {0xB0}},
{0xCC,  10 ,{0x00,0x25,0x0C,0x0A,0x02,0x26,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xC0}},
{0xCC,  6 ,{0x00,0x00,0x00,0x00,0x00,0x00}},
 
{0x00, 1 , {0xCA}},
{0xCC,  5 ,{0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xD0}},
{0xCC,  11 ,{0x00,0x00,0x00,0x00,0x00,0x26,0x01,0x09,0x0b,0x25,0x00}},

{0x00, 1 , {0x00}},
{0xE1,  20 ,{0x00,0x2C,0x35,0x4E,0x68,0x7D,0x84,0xAE,0x98,0xAA,0x60,0x52,0x6C,0x5A,0x60,0x58,0x52,0x43,0x3D,0x00}},                                        

{0x00, 1 , {0x00}},
{0xE2,  20 ,{0x00,0x2C,0x35,0x4E,0x68,0x7D,0x84,0xAE,0x98,0xAA,0x60,0x51,0x6C,0x59,0x60,0x59,0x52,0x44,0x3D,0x00}},
       
{0x00, 1 , {0x80}},
{0xC4,  1 ,{0x30}},
         
{0x00, 1 , {0x98}},
{0xC0,  1 ,{0x00}},
         
{0x00, 1 , {0xa9}},
{0xC0,  1 ,{0x06}},

{0x00, 1 , {0xb0}},
{0xC1,  3 ,{0x20,0x00,0x00}},

{0x00, 1 , {0xe1}},
{0xC0,  2 ,{0x40,0x18}},

{0x00, 1 , {0x80}},
{0xC1,  2 ,{0x03,0x33}},

{0x00, 1 , {0xA0}},
{0xC1,  1 ,{0xe8}},

{0x00, 1 , {0x90}},
{0xb6,  1 ,{0xb4}},

{0x00, 1 , {0xc1}},
{0xF5,  1 ,{0x94}},

{0x00, 1 , {0x00}},
{0xfb,  1 ,{0x01}},

{0x00, 1 , {0x00}},
{0xFF, 3 , {0xFF,0xFF,0xFF}},
   
{0x3A, 1 , {0x77}},

{0x11, 1 , {0x00}},	
{REGFLAG_DELAY, 120, {}},

{0x29, 1 , {0x00}},


//W_COM_1A_0P(0x11); 
//DELAYMS(120);
//{0x00, 1 , {0x00}},
//{0x11, 1 , {0x00}},
//{REGFLAG_DELAY, 120, {}},	
//W_COM_1A_0P(0x29); 
//DELAYMS(10);
//{0x00, 1 , {0x00}},
//{0x29, 1 , {0x00}},
//{REGFLAG_DELAY, 20, {}},	

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//MDELAY(5);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dbi.io_driving_current      = LCM_DRIVING_CURRENT_4MA;
		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;//1
		params->dsi.vertical_backporch					= 18;//10  16
		params->dsi.vertical_frontporch					= 16;//10  15
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 4;//4
		params->dsi.horizontal_backporch				= 57;// 40; 44  50
		params->dsi.horizontal_frontporch				= 46;// 40; 46  48
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		 //params->dsi.horizontal_blanking_pixel				 = 60;



		// Bit rate calculation
		params->dsi.PLL_CLOCK=200;//210
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
		params->dsi.fbk_div =9;
#else
		params->dsi.fbk_div =9;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
/*	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
*/
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/


static unsigned int lcm_compare_id(void)
{

#if 1
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;

	#if defined(BUILD_LK)
		printf("OTM8018B CS uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM8018B CS kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return (LCM_ID_OTM8019A_CS == id)?1:0;
#else
	int adcdata[4];
	int result = 0;
	int lcmid;
		
	IMM_GetOneChannelValue(1,adcdata,&lcmid);
       #if defined(BUILD_LK)
	printf("Otm8018b channelValue=%d\n",lcmid);
	#else 
        printk("Otm8018b channelValue=%d\n",lcmid);
        #endif

	lcmid = lcmid * 1500/4096; //LiuHuojun 20130503 1500?<C1><D9>??
	#if defined(BUILD_LK)
			printf("Otm8018b uboot %s \n", __func__);
			printf("%s lcmid = %d \n", __func__, lcmid);
	#else
			printk("Otm8018b kernel %s \n", __func__);
			printk("%s lcmid = %d \n", __func__, lcmid);
	#endif
	if(lcmid >800 && lcmid<=1200) //add by liutao for lingda:0V tianma:0.7V
	{
			return 1;
	}
	else
	{
			return 0;
	}
#endif
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK

    unsigned char buffer[1];
    unsigned int array[16];

    array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);

      read_reg_v2(0x0A, buffer, 1);

    printk("lcm_esd_check  0x0A = %x\n",buffer[0]);


        if(buffer[0] != 0x9C)  
        {
        return TRUE;
        }

#if 0
	array[0] = 0x00013700;// read id return two byte,version and id
   	dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
	read_reg_v2(0x0D, buffer, 1);
#if defined(BUILD_UBOOT)
    printf("lcm_esd_check 0x0D =%x\n",buffer[0]);
#else
    printk("lcm_esd_check 0x0D =%x\n",buffer[0]);
#endif
   if(buffer[0] != 0x00)
   {
      return TRUE;
   }


   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0E, buffer, 1);
#if defined(BUILD_UBOOT)
    printf("lcm_esd_check  0x0E = %x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0E = %x\n",buffer[0]);
#endif
   if(buffer[0] != 0x00)
   {
      return TRUE;
   }

#endif
    
#endif
    return FALSE;
}


static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK

  unsigned int data_array[16];

    printk("lcm_esd_recover enter \n");
    
    lcm_init();

    data_array[0]=0x00110500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(50);
    
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array, 1, 1);
    
    data_array[0]= 0x00023902;
    data_array[1]= 0xFF51;
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(10);
#endif

    return TRUE;
}


LCM_DRIVER otm8019a_wvga_dsi_vdo_dijing_lcm_drv = 
{
    .name			= "otm8019a_wvga_dsi_vdo_dijing",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
	//.esd_check   = lcm_esd_check,
  //	.esd_recover   = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

