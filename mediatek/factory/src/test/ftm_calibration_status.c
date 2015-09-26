
/*
 * Add for calibration status report
 */
#define TAG "[CalibrationStatus] "

#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <linux/input.h>

#include "common.h"
#include "miniui.h"
#include "ftm.h"

#ifdef FEATURE_FTM_CALIBRATION_STATUS
#define BUF_LEN 128

enum
{
	ITEM_PASS,
	ITEM_FAIL,
};

struct cal_status
{
	struct ftm_module* mod;
	char info[BUF_LEN];
};


#define mod_to_priv(p)  (struct cal_status*)((char*)(p) + sizeof(struct ftm_module))

static item_t cal_status_items[] =
{
	item(ITEM_PASS, uistr_pass), item(ITEM_FAIL, uistr_fail),
	item(-1, NULL),
};

extern int check_modem_calibration(void);

static void dev_info_get_data(struct cal_status* cs)
{
	char* ptr;
	item_t *items;
	int cal_res = 2;

	/* prepare info */
	ptr = cs->info;

	memset(ptr, 0, BUF_LEN);

	cal_res = check_modem_calibration();

	ptr += sprintf(ptr, "%s: %s\n", uistr_calibration_final_test, (cal_res == 1)?uistr_pass:uistr_fail);	/* Wait API	 */

	return;
}

int cal_status_entry(struct ftm_param* param, void* priv)
{
	text_t title;
	struct itemview* iv;
	text_t text;
	bool exit = false;
	int chosen;
	struct cal_status* cs = (struct cal_status*) priv;

	//PRINTF("\n");
	//assert(cs != NULL);
	if(cs == NULL)
		return -1;
	// step 1: 设置标题内容颜色和文字内容颜色
	init_text(&title, uistr_calibration_status, COLOR_YELLOW);
	init_text(&text, cs->info, COLOR_YELLOW);

	dev_info_get_data(cs);

	// step 2: 初始化菜单选择项
	iv = ui_new_itemview();

	// step 3:
	iv->set_title(iv, &title);
	iv->set_items(iv, cal_status_items, 0);
	iv->set_text(iv, &text);

	do
	{
		// step 4: 监控用户的按键选择
		chosen = iv->run(iv, &exit);
		switch (chosen)
		{
		case ITEM_PASS:
		case ITEM_FAIL:
			if (chosen == ITEM_PASS)
			{
				cs->mod->test_result = FTM_TEST_PASS;
			}
			else if (chosen == ITEM_FAIL)
			{
				cs->mod->test_result = FTM_TEST_FAIL;
			}
			exit = true;
			break;
		}
		if (exit)
		{
			//PRINTF("\n");
			break;
		}
	}
	while (1);

	//PRINTF("\n");

	return 0;
}

int cal_status_init(void)
{
	int ret = 0;

	struct ftm_module* mod;
	struct cal_status* cs = NULL;

	//PRINTF("\n");
	LOGD(TAG "%s\n", __FUNCTION__);
	mod = ftm_alloc(ITEM_CALIBRATION_STATUS, sizeof(struct cal_status));
	if (!mod)
		return -ENOMEM;

	cs = mod_to_priv(mod);
	cs->mod = mod;
	ret = ftm_register(mod, cal_status_entry, (void *) cs);
	//PRINTF("\n");

	return 0;
}
#endif /* FACTORY_CKT_STANDARD */

