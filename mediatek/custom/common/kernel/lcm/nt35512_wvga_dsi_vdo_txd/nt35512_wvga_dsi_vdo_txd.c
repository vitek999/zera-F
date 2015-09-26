#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

static bool lcm_is_init = false;
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
};
#ifdef BUILD_LK
#include  <platform/mt_pmic.h>
void dct_pmic_VGP2_enable(bool dctEnable)
{
	pmic_config_interface(DIGLDO_CON29, 0x5, PMIC_RG_VGP2_VOSEL_MASK, PMIC_RG_VGP2_VOSEL_SHIFT); // 2.8v ËÕ ÓÂ 2013Äê10ÔÂ31ÈÕ 17:55:43
	pmic_config_interface( (U32)(DIGLDO_CON8),
                             (U32)(dctEnable),
                             (U32)(PMIC_RG_VGP2_EN_MASK),
                             (U32)(PMIC_RG_VGP2_EN_SHIFT)
	                         );
}
#else
void dct_pmic_VGP2_enable(bool dctEnable);
#endif


extern void DSI_clk_ULP_mode(bool enter);
extern void DSI_lane0_ULP_mode(bool enter);
void DSI_Enter_ULPM(bool enter)
{
	DSI_clk_ULP_mode(enter);  //enter ULPM
	DSI_lane0_ULP_mode(enter);
}

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
	//{0xFF,	4,	{0xAA, 0x55, 0xA5, 0x80}},
	//{0xF7,	15,	{0x63, 0x40, 0x00, 0x00, 0x00, 0x01, 0xC4, 0xA2, 0x00, 0x02,0x64,0x54,0x48,0x00,0xD0}},
	//{0xFF,	4,	{0xAA, 0x55, 0xA5, 0x00}},

	#if 1
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x01}},
	//{0xc0,	2,	{0x04,0x08}},
	{0xB0,	1,	{0x0D}},
	{0xB6,	1,	{0x36}},
	{0xB1,	1,	{0x0D}},
	{0xB7,	1,	{0x36}},
	
	{0xB2,	1,	{0x00}},
	//{0xB7,	1,	{0x14}},
	{0xB8,	1,	{0x26}},
	{0xB9,	1,	{0x36}},
	{0xB5,	1,	{0x08}},
	{0xC2,	1,	{0x03}},
       {0xBA,	1,	{0x26}},
       {0xBC,	3,	{0x00,0x80,0x00}},
       {0xBD,	3,	{0x00,0x80,0x00}},
       
	//{0xC3,	1,	{0x04}},
	//{0xC2,	1,	{0x00}},
	//{0xC0,	2,	{0x04,0x08}},

	//{0xBC,	3,	{0x00,0x70,0x00}},
	//{0xBD,	3,	{0x00,0x70,0x00}},
	{0xBE,	2,	{0x00,0x40}},
		
#if  0
	{0xD1,	52,	{0x00,0x2D,0x00,0x3C,0x00,0x58,0x00,0x6F,0x00,0x84,0x00,0xA6,0x00,0xC2,0x00,0xF3,0x01,0x1A,0x01,0x58,0x01,0x8A,0x01,0xD9,0x02,0x1A,0x02,0x1C,0x02,0x56,0x02,0x94,0x02,0xB8,0x02,0xE6,0x03,0x04,0x03,0x29,0x03,0x3F,0x03,0x58,0x03,0x65,0x03,0x97,0x03,0xF6,0x03,0xFF}},
	{0xD2,	52, {0x00,0x2D,0x00,0x3C,0x00,0x58,0x00,0x6F,0x00,0x84,0x00,0xA6,0x00,0xC2,0x00,0xF3,0x01,0x1A,0x01,0x58,0x01,0x8A,0x01,0xD9,0x02,0x1A,0x02,0x1C,0x02,0x56,0x02,0x94,0x02,0xB8,0x02,0xE6,0x03,0x04,0x03,0x29,0x03,0x3F,0x03,0x58,0x03,0x65,0x03,0x97,0x03,0xF6,0x03,0xFF}},
	{0xD3,	52, {0x00,0x2D,0x00,0x3C,0x00,0x58,0x00,0x6F,0x00,0x84,0x00,0xA6,0x00,0xC2,0x00,0xF3,0x01,0x1A,0x01,0x58,0x01,0x8A,0x01,0xD9,0x02,0x1A,0x02,0x1C,0x02,0x56,0x02,0x94,0x02,0xB8,0x02,0xE6,0x03,0x04,0x03,0x29,0x03,0x3F,0x03,0x58,0x03,0x65,0x03,0x97,0x03,0xF6,0x03,0xFF}},
	{0xD4,	52, {0x00,0x2D,0x00,0x3C,0x00,0x58,0x00,0x6F,0x00,0x84,0x00,0xA6,0x00,0xC2,0x00,0xF3,0x01,0x1A,0x01,0x58,0x01,0x8A,0x01,0xD9,0x02,0x1A,0x02,0x1C,0x02,0x56,0x02,0x94,0x02,0xB8,0x02,0xE6,0x03,0x04,0x03,0x29,0x03,0x3F,0x03,0x58,0x03,0x65,0x03,0x97,0x03,0xF6,0x03,0xFF}},
	{0xD5,	52, {0x00,0x2D,0x00,0x3C,0x00,0x58,0x00,0x6F,0x00,0x84,0x00,0xA6,0x00,0xC2,0x00,0xF3,0x01,0x1A,0x01,0x58,0x01,0x8A,0x01,0xD9,0x02,0x1A,0x02,0x1C,0x02,0x56,0x02,0x94,0x02,0xB8,0x02,0xE6,0x03,0x04,0x03,0x29,0x03,0x3F,0x03,0x58,0x03,0x65,0x03,0x97,0x03,0xF6,0x03,0xFF}},
	{0xD6,	52, {0x00,0x2D,0x00,0x3C,0x00,0x58,0x00,0x6F,0x00,0x84,0x00,0xA6,0x00,0xC2,0x00,0xF3,0x01,0x1A,0x01,0x58,0x01,0x8A,0x01,0xD9,0x02,0x1A,0x02,0x1C,0x02,0x56,0x02,0x94,0x02,0xB8,0x02,0xE6,0x03,0x04,0x03,0x29,0x03,0x3F,0x03,0x58,0x03,0x65,0x03,0x97,0x03,0xF6,0x03,0xFF}},
#endif 

	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},

	{0xB1,	2,	{0xF8,0x06}},
	{0xB6,	1,	{0x0A}},//SDT: 
	{0xB7,	2,	{0x00,0x00}},
	{0xB8,	4,	{0x01,0x05,0x05,0x05}},//Source EQ:
	{0xBC,	3,	{0x00,0x00,0x00}},
	{0xB1,	3,	{0x63, 0x00, 0x01}},
	{0xCC,	3,	{0x03,0x00,0x00}},
       {0xBA,	1,	{0x01}},
       {0x35,	1,	{0x00}},
       {0x3A,	1,	{0x77}},
       {0xBA,	1,	{0x01}},
       {0x11,	0,	{0x00}},
      {REGFLAG_DELAY, 120, {}},
	{0x29,	0,	{0x00}},
	//{0x21,	0,	{0x00}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

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
				//MDELAY(10);//soso add or it will fail to send register
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


		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 0x05;// 3    2
		params->dsi.vertical_backporch					= 0x0d;// 20   1
		params->dsi.vertical_frontporch					= 0x08; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 0x12;// 50  2
		params->dsi.horizontal_backporch				= 0x5f;
		params->dsi.horizontal_frontporch				= 0x5f;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

              params->dsi.PLL_CLOCK = 210;
		// Bit rate calculation
		//params->dsi.pll_div1=30;//32		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
}

static void lcm_init(void)
{
          unsigned int data_array[64];
		//SET_RESET_PIN(0);
		//SET_RESET_PIN(1);
		//lcm_is_init = true;
    SET_RESET_PIN(1);
      MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);
#if 1

	data_array[0] = 0x00063902;
	data_array[1] = 0x52AA55F0;//0Xf0 0x55, 0xAA, 0x52  page 1
	data_array[2] = 0x00000108;//0x08, 0x01
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x0DB01500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x36B61500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0DB11500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x36B71500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00B21500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x26B81500;dsi_set_cmdq(data_array, 1, 1);
       data_array[0] = 0x36B91500;dsi_set_cmdq(data_array, 1, 1);
       data_array[0] = 0x08B51500;dsi_set_cmdq(data_array, 1, 1);
	   
	
	//data_array[0] = 0x14B61500;dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x14B71500;dsi_set_cmdq(data_array, 1, 1);
	///data_array[0] = 0x24B81500;dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x34B91500;dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x14BA1500;dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x01BF1500;dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x04C31500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03C21500;dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x26BA1500;dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x00033902;
	//data_array[1] = 0x000804C0;
	//dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008000BC;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008000BD;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00033902;
	data_array[1] = 0x004000BE;
	dsi_set_cmdq(data_array, 2, 1);

	//Gamma
	data_array[0] = 0x00353902;
	data_array[1] = 0x002D00D1;//D1,0x00,0x00,0x00,
	data_array[2] = 0x0058003c;//0x0C,0x00,0x22,0x00
	data_array[3] = 0x0084006f;//0x36,0x00,0x47,0x00,
	data_array[4] = 0x00c200a6;//0x66,0x00,0x82,0x00
	data_array[5] = 0x011A01f3;//0xB2,0x00,0xDA,0x01
	data_array[6] = 0x018a0158;//0x1B,0x01,0x51,0x01
	data_array[7] = 0x021a02d9;//0xA5,0x01,0xED,0x01
	data_array[8] = 0x0256021c;//0xEE,0x02,0x30,0x02
	data_array[9] = 0x02b80294;//0x75,0x02,0x9E,0x02
	data_array[10] = 0x030403e6;//,0xD7,0x02,0xF9,0x03
	data_array[11] = 0x033f0329;//0x28,0x03,0x47,0x03
	data_array[12] = 0x03650358;//0x6C,0x03,0x81,0x03
	data_array[13] = 0x03f60397;//0x90,0x03,0xC0,0x03,
	data_array[14] = 0x000000ff;//0xF8
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0] = 0x00353902;
	data_array[1] = 0x002D00D2;//D1,0x00,0x00,0x00,
	data_array[2] = 0x0058003c;//0x0C,0x00,0x22,0x00
	data_array[3] = 0x0084006f;//0x36,0x00,0x47,0x00,
	data_array[4] = 0x00c200a6;//0x66,0x00,0x82,0x00
	data_array[5] = 0x011A01f3;//0xB2,0x00,0xDA,0x01
	data_array[6] = 0x018a0158;//0x1B,0x01,0x51,0x01
	data_array[7] = 0x021a02d9;//0xA5,0x01,0xED,0x01
	data_array[8] = 0x0256021c;//0xEE,0x02,0x30,0x02
	data_array[9] = 0x02b80294;//0x75,0x02,0x9E,0x02
	data_array[10] = 0x030403e6;//,0xD7,0x02,0xF9,0x03
	data_array[11] = 0x033f0329;//0x28,0x03,0x47,0x03
	data_array[12] = 0x03650358;//0x6C,0x03,0x81,0x03
	data_array[13] = 0x03f60397;//0x90,0x03,0xC0,0x03,
	data_array[14] = 0x000000ff;//0xF8
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0] = 0x00353902;
	data_array[1] = 0x002D00D3;//D1,0x00,0x00,0x00,
	data_array[2] = 0x0058003c;//0x0C,0x00,0x22,0x00
	data_array[3] = 0x0084006f;//0x36,0x00,0x47,0x00,
	data_array[4] = 0x00c200a6;//0x66,0x00,0x82,0x00
	data_array[5] = 0x011A01f3;//0xB2,0x00,0xDA,0x01
	data_array[6] = 0x018a0158;//0x1B,0x01,0x51,0x01
	data_array[7] = 0x021a02d9;//0xA5,0x01,0xED,0x01
	data_array[8] = 0x0256021c;//0xEE,0x02,0x30,0x02
	data_array[9] = 0x02b80294;//0x75,0x02,0x9E,0x02
	data_array[10] = 0x030403e6;//,0xD7,0x02,0xF9,0x03
	data_array[11] = 0x033f0329;//0x28,0x03,0x47,0x03
	data_array[12] = 0x03650358;//0x6C,0x03,0x81,0x03
	data_array[13] = 0x03f60397;//0x90,0x03,0xC0,0x03,
	data_array[14] = 0x000000ff;//0xF8
	dsi_set_cmdq(data_array, 15, 1);


	data_array[0] = 0x00353902;
	data_array[1] = 0x002D00D4;//D1,0x00,0x00,0x00,
	data_array[2] = 0x0058003c;//0x0C,0x00,0x22,0x00
	data_array[3] = 0x0084006f;//0x36,0x00,0x47,0x00,
	data_array[4] = 0x00c200a6;//0x66,0x00,0x82,0x00
	data_array[5] = 0x011A01f3;//0xB2,0x00,0xDA,0x01
	data_array[6] = 0x018a0158;//0x1B,0x01,0x51,0x01
	data_array[7] = 0x021a02d9;//0xA5,0x01,0xED,0x01
	data_array[8] = 0x0256021c;//0xEE,0x02,0x30,0x02
	data_array[9] = 0x02b80294;//0x75,0x02,0x9E,0x02
	data_array[10] = 0x030403e6;//,0xD7,0x02,0xF9,0x03
	data_array[11] = 0x033f0329;//0x28,0x03,0x47,0x03
	data_array[12] = 0x03650358;//0x6C,0x03,0x81,0x03
	data_array[13] = 0x03f60397;//0x90,0x03,0xC0,0x03,
	data_array[14] = 0x000000ff;//0xF8
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0] = 0x00353902;
	data_array[1] = 0x002D00D5;//D1,0x00,0x00,0x00,
	data_array[2] = 0x0058003c;//0x0C,0x00,0x22,0x00
	data_array[3] = 0x0084006f;//0x36,0x00,0x47,0x00,
	data_array[4] = 0x00c200a6;//0x66,0x00,0x82,0x00
	data_array[5] = 0x011A01f3;//0xB2,0x00,0xDA,0x01
	data_array[6] = 0x018a0158;//0x1B,0x01,0x51,0x01
	data_array[7] = 0x021a02d9;//0xA5,0x01,0xED,0x01
	data_array[8] = 0x0256021c;//0xEE,0x02,0x30,0x02
	data_array[9] = 0x02b80294;//0x75,0x02,0x9E,0x02
	data_array[10] = 0x030403e6;//,0xD7,0x02,0xF9,0x03
	data_array[11] = 0x033f0329;//0x28,0x03,0x47,0x03
	data_array[12] = 0x03650358;//0x6C,0x03,0x81,0x03
	data_array[13] = 0x03f60397;//0x90,0x03,0xC0,0x03,
	data_array[14] = 0x000000ff;//0xF8
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0] = 0x00353902;
	data_array[1] = 0x002D00D6;//D1,0x00,0x00,0x00,
	data_array[2] = 0x0058003c;//0x0C,0x00,0x22,0x00
	data_array[3] = 0x0084006f;//0x36,0x00,0x47,0x00,
	data_array[4] = 0x00c200a6;//0x66,0x00,0x82,0x00
	data_array[5] = 0x011A01f3;//0xB2,0x00,0xDA,0x01
	data_array[6] = 0x018a0158;//0x1B,0x01,0x51,0x01
	data_array[7] = 0x021a02d9;//0xA5,0x01,0xED,0x01
	data_array[8] = 0x0256021c;//0xEE,0x02,0x30,0x02
	data_array[9] = 0x02b80294;//0x75,0x02,0x9E,0x02
	data_array[10] = 0x030403e6;//,0xD7,0x02,0xF9,0x03
	data_array[11] = 0x033f0329;//0x28,0x03,0x47,0x03
	data_array[12] = 0x03650358;//0x6C,0x03,0x81,0x03
	data_array[13] = 0x03f60397;//0x90,0x03,0xC0,0x03,
	data_array[14] = 0x000000ff;//0xF8
	dsi_set_cmdq(data_array, 15, 1);
	//Gamma

	data_array[0] = 0x00063902;
	data_array[1] = 0x52AA55F0;   //page 0
	data_array[2] = 0x00000008;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00033902;
	data_array[1] = 0x0000f8B1;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0aB61500;   //color enhance
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x05B61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00033902;
	data_array[1] = 0x000000B7;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00053902;
	data_array[1] = 0x050501B8;
	data_array[2] = 0x00000005;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x000000Bc;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x000003cc;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x01Ba1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;    // 2-dot
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x773a1500;    // 2-dot
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
       //PacketHeader[05 29 00 xx] // Display On 
       //Delay 10ms 
    data_array[0] = 0x00290500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);

#else
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#endif

}


static void lcm_suspend(void)
{
	#if 0
	DSI_Enter_ULPM(1); /* Enter low power mode  */
	MDELAY(60);
	mt_set_gpio_out(GPIO112,GPIO_OUT_ZERO);
	MDELAY(150);
	dct_pmic_VGP2_enable(0); /* [Ted 5-28] Disable VCI power to prevent lcd polarization */
	lcm_is_init = false;
	#else
 	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	#endif
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    static int count = 0;
    static int err_count = 0;
    static int uncount = 0;
    int i;
    unsigned char fResult;
    unsigned char buffer[12];
    unsigned int array[16];

#ifdef ESD_DEBUG
    printk("lcm_esd_check <<<\n");
#endif
    for (i = 0; i < 12; i++)
        buffer[i] = 0x00;

    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);

    //---------------------------------
    // Read [9Ch, 00h, ECC] + Error Report(4 Bytes)
    //---------------------------------
    read_reg_v2(0x0A, buffer, 1);

#ifdef ESD_DEBUG
    printk("lcm_esd_check : read(0x0A) : [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);
#endif

    //---------------------------------
    // Judge Readout & Error Report
    //---------------------------------
    #if 0
    if (buffer[3] == 0x02) // Check data identifier of error report
    {
        if (buffer[4] & 0x02) // Check SOT sync error
            err_count ++;
        else
            err_count = 0;
    }
    else
    {
        err_count = 0;
    }
#endif
#ifdef ESD_DEBUG
    printk("lcm_esd_check err_count = %d\n", err_count);
#endif
    if ((buffer[0] != 0x9C) || (err_count >= 2))
    {
        err_count = 0;
        uncount++;

#if 0//ndef BUILD_LK
        printk("lcm_esd_check failed, err_count = %d\n", err_count);
        for (i = 0; i < 7; i++)
	        printk("buffer[%d] : 0x%x \n", i, buffer[i]);
#endif

#ifdef ESD_DEBUG
        printk("lcm_esd_check unnormal uncount = %d\n", uncount);
        printk("lcm_esd_check >>>\n");
#endif
        fResult = 1;
    }
    else
    {
        count ++;
#ifdef ESD_DEBUG
        printk("lcm_esd_check normal count = %d\n", count);
        printk("lcm_esd_check >>>\n");
#endif
        fResult = 0;
    }

#if 0
        printk("lcm_esd_check lcm_esd_test:%d\n",lcm_esd_test);
        if(lcm_esd_test==20)
        {
            lcm_esd_test = 0;
            return TRUE;
        }
		lcm_esd_test++;
	return FALSE;
#endif

    if (fResult)
        return TRUE;
    else
        return FALSE;
#endif
} 

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
    static int recount = 0;

#ifdef ESD_DEBUG
    printk("lcm_esd_recover\n");
#endif

    lcm_init();
    recount ++;


    //printk("lcm_esd_recover recover recount = %d\n", recount);

    return TRUE;
#endif
}

unsigned char buffer_temp[2];
static void lcm_resume(void)
{
	#if 0
	if(!lcm_is_init)
		lcm_init();
	#else
	//lcm_compare_id();
	#ifndef BUILD_LK
printk("\n\n\n\n[35512]%s, id0 = 0x%08x,id1 = 0x%08x\n", __func__, buffer_temp[0],buffer_temp[1]);
	#endif
	lcm_init();
	
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	#endif
}
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int array[16];
       unsigned int data_array[10];
#if 0
    dct_pmic_VGP2_enable(1);
    MDELAY(5);
    //SET_RESET_PIN(1);
    mt_set_gpio_mode(GPIO112,GPIO_MODE_00);
    mt_set_gpio_dir(GPIO112,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO112,GPIO_OUT_ONE);
    MDELAY(5);
    //SET_RESET_PIN(0);
    mt_set_gpio_out(GPIO112,GPIO_OUT_ZERO);
    MDELAY(5);
    //SET_RESET_PIN(1);
    mt_set_gpio_out(GPIO112,GPIO_OUT_ONE);
    MDELAY(5);
#endif
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);
	data_array[0] = 0x00063902;
	data_array[1] = 0x52AA55F0;   //page 1
	data_array[2] = 0x00000108;
	dsi_set_cmdq(data_array, 3, 1);
	
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xc5, buffer, 2);
	//id = buffer[1]; //we only need ID
	//buffer_temp[0]=buffer[0];
	//buffer_temp[1] = buffer[1];
#if defined(BUILD_LK)
	/*The Default Value should be 0x00,0x80,0x00*/
	printf("\n\n\n\n[soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif
  if((buffer[0]==0x55)&&(buffer[1]==0x12))
    return 1;//(id == 0x80)?1:0;
    else
    return 0;
}


LCM_DRIVER nt35512_wvga_dsi_vdo_txd_lcm_drv = 
{
    .name			= "nt35512_wvga_dsi_vdo_txd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.esd_check   = lcm_esd_check,
	.esd_recover   = lcm_esd_recover,
	.compare_id    = lcm_compare_id,
};

