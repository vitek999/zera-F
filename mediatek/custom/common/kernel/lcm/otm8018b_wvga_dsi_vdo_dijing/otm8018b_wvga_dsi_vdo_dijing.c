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

#define LCM_ID_OTM8018B	0x8009

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

static LCM_UTIL_FUNCS lcm_util = {0};

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
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
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

	{0x00,	1,	{0x00}},
       {0xff,	3,	{0x80, 0x09, 0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x80, 0x09}},

	{0x00,	1,	{0x03}},
       {0xFF,	1,	{0x01}},
       
	{0x00,	1,	{0x80}},
       {0xC4,	1,	{0x30}},
	{REGFLAG_DELAY, 10, {}},
	{0x00,	1,	{0x8A}},
	{0xC4,	1,	{0x40}},
	{REGFLAG_DELAY, 10, {}},

	//{0x00,	1,	{0xB2}},//add
	//{0xF5,	4,	{0x15,0x00,0x15,0x00}},
	//{0xff,	1,	{0x01}},
	{0x00,	1,	{0x8B}},
	{0xB0,	1,	{0x40}},
	
	//{0x00,	1,	{0xC6}},
	//{0xB0,	1,	{0x03}},
	
	//{0x00,	1,	{0x90}},
	//{0xB3,	1,	{0x02}},
	
	//{0x00,	1,	{0x92}},
	//{0xB3,	1,	{0x45}},
	
	{0x00,	1,	{0xA6}},
	{0xB3,	2,	{0x20,0x01}},
	{0x00,	1,	{0x90}},
	{0xC0,	6,	{0x00,0x44,0x00,0x00,0x00,0x03}},
	{0x00,	1,	{0xB4}},
	{0xC0,	1,	{0x10}},//0X08
	{0x00,	1,	{0x81}},
	{0xC1,	1,	{0x66}},
	{0x00,	1,	{0xA0}},
	{0xC1,	1,	{0x02}},
	//{0x00,	1,	{0xA0}},
	//{0xC1,	1,	{0xEA}},
	{0x00,	1,	{0xA6}},
	{0xC1,	3,	{0x01,0x00,0x00}},
	{0x00,	1,	{0x81}},
	{0xC4,	1,	{0x81}},
	{0x00,	1,	{0x87}},
	{0xC4,	1,	{0x00}},
	{0x00,	1,	{0x89}},
	{0xC4,	1,	{0x00}},
	{0x00,	1,	{0x82}},
	{0xC5,	1,	{0x83}},
	{0x00,	1,	{0x90}},
	{0xC5,	7,	{0x96,0x34,0x06,0x91,0x33,0x34,0x23}},
	{0x00,	1,	{0xB1}},
	{0xC5,	1,	{0xA8}},
	{0x00,	1,	{0xC0}},
	{0xC5,	1,	{0x00}},
       {0x00,	1,	{0x00}},
	{0xD8,	2,	{0x6F,0x6F}},
       {0x00,	1,	{0x00}},
	{0xD9,	1,	{0x2A}},
       {0x00,	1,	{0xB2}},
	{0xF5,	4,	{0x15,0x00,0x15,0x00}},
	
	{0x00,	1,	{0x80}},
	{0xCE,	12,	{0x86,0x01,0x00,0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xA0}},
	{0xCE,	14,	{0x18,0x05,0x83,0x39,0x00,0x00,0x00,0x18,0x04,0x83,0x3A,0x00,0x00,0x00}},
	{0x00,	1,	{0xB0}},
	{0xCE,	14,	{0x18,0x03,0x83,0x3B,0x86,0x00,0x00,0x18,0x02,0x83,0x3C,0x88,0x00,0x00}},
	{0x00,	1,	{0xC0}},
	{0xCF,	10,	{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x02,0x00,0x00}},
	{0x00,	1,	{0xD0}},
	{0xCF,	1,	{0x00}},
	{0x00,	1,	{0xC0}},
	{0xCB,	15,	{0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xD0}},
	{0xCB,	15,	{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xE0}},
	{0xCB,	10,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	//{0x00,	1,	{0xF0}},
	//{0xCB,	10,	{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},
	{0x00,	1,	{0x80}},
	{0xCC,	10,	{0x00,0x26,0x09,0x0B,0x01,0x25,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0x90}},
	{0xCC,	15,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x0A,0x0C,0x02}},
	{0x00,	1,	{0xA0}},
	{0xCC,	15,	{0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xB0}},
	{0xCC,	10,	{0x00,0x25,0x0C,0x0A,0x02,0x26,0x00,0x00,0x00,0x00}},
	{0x00,	1,	{0xC0}},
	{0xCC,	15,	{0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x0B,0x09,0x01}},
	{0x00,	1,	{0xD0}},
	{0xCC,	15,	{0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x0B,0x09,0x01}},
	//{0x00,	1,	{0x00}},
	//{0xD8,	2,	{0x77,0x77}},
	//{0x00,	1,	{0x00}},
	//{0x00,	1,	{0x00}},
	//{0xD9,	1,	{0x48}},
	//{REGFLAG_DELAY, 200, {}},
	{0x00,	1,	{0x00}},
	//{REGFLAG_DELAY, 50, {}},
	{0xE1,	16,	{0x08,0x0A,0x0B,0x0A,0x04,0x1A,0x0F,0x0F,0x00,0x04,0x02,0x06,0x0E,0x26,0x22,0x1D}},
	{0x00,	1,	{0x00}},
	//{REGFLAG_DELAY, 50, {}},
	{0xE2,	16,	{0x08,0x0A,0x0B,0x0A,0x05,0x1A,0x0F,0x0F,0x00,0x04,0x01,0x06,0x0D,0x25,0x23,0x1D}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
       {0x00,	1,	{0xA0}},
	{0xC1,	1,	{0x02}},

       {0x00,	1,	{0xA6}},
	{0xC1,	3,	{0x01,0x00,0x00}},

       {0x00,	1,	{0xC6}},
	{0xB0,	1,	{0x03}},

       {0x00,	1,	{0x81}},
	{0xC5,	1,	{0x66}},

       {0x00,	1,	{0xB6}},
	{0xF5,	1,	{0x06}},

       {0x00,	1,	{0x8B}},
	{0xB0,	1,	{0x40}},

       {0x00,	1,	{0xB1}},
	{0xB0,	1,	{0x80}},

       {0x00,	1,	{0x00}},
	{0xFF,	3,	{0xFF,0xFF,0xFF}},
	
       {0x3A,	1,	{0x77}},
       
      {0x11,	0,	{}},
      {REGFLAG_DELAY, 120, {}},

     {0x29,	0,	{}},
     {REGFLAG_DELAY, 10, {}},
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

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

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
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

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;
		params->dsi.horizontal_backporch				= 37;
		params->dsi.horizontal_frontporch				= 37;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.PLL_CLOCK=200;
#if 0
		// Bit rate calculation
		params->dsi.pll_div1=29;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames
		#endif
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
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

	#if defined(BUILD_UBOOT)
		//printf("OTM8018B uboot %s \n", __func__);
		//printf("%s id = 0x%08x \n", __func__, id);
	#else
		//printk("OTM8018B kernel %s \n", __func__);
		//printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return (LCM_ID_OTM8018B == id)?1:0;
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
    printk("lcm_esd_check sen: read(0x0A) : [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);
#endif

    //---------------------------------
    // Judge Readout & Error Report
    //---------------------------------

#ifdef ESD_DEBUG
    printk("lcm_esd_check err_count = %d\n", err_count);
#endif
    if ((buffer[0] != 0x9C) || (err_count >= 2))
    {
        err_count = 0;
        uncount++;

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



LCM_DRIVER otm8018b_wvga_dsi_vdo_dijing_lcm_drv = 
{
    .name			= "otm8018b_wvga_dsi_vdo_dijing",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.esd_check   = lcm_esd_check,
	.esd_recover   = lcm_esd_recover,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

