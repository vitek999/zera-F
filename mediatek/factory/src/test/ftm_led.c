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

#include "cust.h"
#include "common.h"
#include "miniui.h"
#include "ftm.h"

#ifdef FEATURE_FTM_LED

//#define RGB_LED_ALONE_TEST // ckt LiuHuojun 2013.3.06 15:16 RGB LED单独测试

#define RED_LED_FILE		"/sys/class/leds/red/brightness"
#define GREEN_LED_FILE		"/sys/class/leds/green/brightness"
#define BLUE_LED_FILE		"/sys/class/leds/blue/brightness"
#define BUTTON_LED_FILE	    "/sys/class/leds/button-backlight/brightness"
#define KEYPAD_LED_FILE	    "/sys/class/leds/keypad-backlight/brightness"

// 苏 勇 2013年11月27日 10:12:09bool led_test_exit = false;
// 苏 勇 2013年11月27日 10:12:09static bool led_thread_exit = false;
// 苏 勇 2013年11月27日 10:12:09static bool led_thread_start = false;


bool keypadled_test_exit = false;
static bool keypadled_thread_exit = false;
static bool keypadled_thread_start = false;

// 苏 勇 2013年11月27日 10:10:54static pthread_t led_thread;
// 苏 勇 2013年11月27日 10:10:54static pthread_t keypadled_thread;


#if defined (CUST_LED_HAVE_BUTTON_BACKLIGHT)
#define CUST_HAVE_KEYPAD_LED
#elif defined (CUST_LED_HAVE_KEYPAD_BACKLIGHT)
#define CUST_HAVE_KEYPAD_LED
#endif

static char *led_seq[] = {
#ifdef CUST_LED_HAVE_RED
	RED_LED_FILE,
#endif
#ifdef CUST_LED_HAVE_GREEN
	GREEN_LED_FILE,
#endif
#ifdef CUST_LED_HAVE_BLUE
	BLUE_LED_FILE,
#endif
	NULL
};

static char *keypadled_seq[] = {
#ifdef CUST_LED_HAVE_BUTTON_BACKLIGHT
	BUTTON_LED_FILE,
#endif
#ifdef CUST_LED_HAVE_KEYPAD_BACKLIGHT
	KEYPAD_LED_FILE,
#endif
	NULL
};


enum {
	#if defined(RGB_LED_ALONE_TEST)
	ITEM_RLED_TEST,
	ITEM_GLED_TEST,
	ITEM_BLED_TEST,
	#else
 ITEM_NLED_TEST,
	#endif
	ITEM_KEYPAD_LED_TEST,
	ITEM_PASS,
	ITEM_FAIL,
};

static item_t items[] = {

// 苏 勇 2013年11月27日 10:33:31	#ifndef CUST_HAVE_NLEDS
	#ifdef CUST_HAVE_NLEDS
	#if defined(RGB_LED_ALONE_TEST)
  item(ITEM_RLED_TEST, uistr_info_ledR_test),
  item(ITEM_GLED_TEST, uistr_info_ledG_test),
  item(ITEM_BLED_TEST, uistr_info_ledB_test),
	#else
	//item(ITEM_NLED_TEST, uistr_info_nled_test),
	#endif
	#endif
		
	#ifdef CUST_HAVE_KEYPAD_LED
	item(ITEM_KEYPAD_LED_TEST,  uistr_info_keypad_led_test),
	#endif
	
	item(ITEM_PASS,   uistr_pass),
	item(ITEM_FAIL,   uistr_fail),
	item(-1, NULL),
};

#if defined(RGB_LED_ALONE_TEST)
static int led_inx=1; // ckt LiuHuojun 2013.3.06 15:24 指示当前测试的是哪个LED
static int led_inx_pre=0;
#define MAX_LED (int)(sizeof(led_seq)/sizeof(led_seq[0]))

static pthread_t led_thread[MAX_LED];
static pthread_t keypadled_thread;

bool led_test_exit[MAX_LED] = {false};
static bool led_thread_exit[MAX_LED] = {false};
static bool led_thread_start[MAX_LED] = {false};
#else
static pthread_t led_thread;
static pthread_t keypadled_thread;

bool led_test_exit = false;
static bool led_thread_exit = false;
static bool led_thread_start = false;
#endif


extern int status;
static int
write_int(char const* path, int value)
{
	int fd;

	if (path == NULL)
		return -1;

	fd = open(path, O_RDWR);
	if (fd >= 0) {
		char buffer[20];
		int bytes = sprintf(buffer, "%d\n", value);
		int amt = write(fd, buffer, bytes);
		close(fd);
		if(amt == -1)
		{
			LOGE("write_int failed to write %s\n", path);
			return -errno;
		}
		else
		{
			//LOGD("write_int write %s:%d OK!\n", path, value);
			return 0;
		}
		//return amt == -1 ? -errno : 0;
	}

	LOGE("write_int failed to open %s\n", path);
	return -errno;
}

#if defined(RGB_LED_ALONE_TEST)
static void *update_RGBled_thread(void *priv)
{
	int index = 0;

	// no LED for test
	if (led_seq[0] == NULL) {
		pthread_exit(NULL);
		return NULL;
	}

	LOGD("%s: Start:[%d],status:%d\n", __FUNCTION__,led_inx,status);
	led_thread_start[led_inx-1] = true;
	led_thread_exit[led_inx-1] = false;

if(status == 1)
{
	do {
		write_int(led_seq[index], 255);
		if (led_test_exit[led_inx-1])
			break;
		sleep(1);
		if (led_test_exit[led_inx-1])
			break;
		write_int(led_seq[index], 0);
		sleep(1);
	
		if (led_seq[++index] == NULL)
			index = 0;
		
	} while (1);

	
	// switch all leds off
	while(led_seq[index] != NULL)
		write_int(led_seq[index++], 0);
}
else
{
 write_int(led_seq[led_inx-1], 255);
	do {
		if (led_test_exit[led_inx-1])
			break;
		sleep(1);
	} while (1);

	// switch all leds off
	index = 0;
	while(led_seq[index] != NULL)
		write_int(led_seq[index++], 0);
}

	led_thread_exit[led_inx-1] = true;
	led_thread_start[led_inx-1] = false;
	LOGD("%s: Exit:[%d]\n", __FUNCTION__,index);
	pthread_exit(NULL);

	return NULL;
}

#else

static void *update_led_thread(void *priv)
{
	int index = 0;
if(status == 0){

	// no LED for test
	if (led_seq[0] == NULL) {
		pthread_exit(NULL);
		return NULL;
	}

	LOGD("%s: Start\n", __FUNCTION__);
	led_thread_start = true;
	led_thread_exit = false;

	do {
		write_int(led_seq[index], 255);
		if (led_test_exit)
			break;
		sleep(1);
		if (led_test_exit)
			break;
		write_int(led_seq[index], 0);
		sleep(1);
	
		if (led_seq[++index] == NULL)
			index = 0;
		
	} while (1);

	
	// switch all leds off
	while(led_seq[index] != NULL)
		write_int(led_seq[index++], 0);
	
	led_thread_exit = true;
	led_thread_start = false;
	LOGD("%s: Exit\n", __FUNCTION__);
	pthread_exit(NULL);
}
else if(status == 1) {
	// no LED for test
	if (led_seq[0] == NULL) {
		pthread_exit(NULL);
		return NULL;
	}

	LOGD("%s: Start\n", __FUNCTION__);
	led_thread_start = true;
	led_thread_exit = false;

	do {
		write_int(led_seq[index], 255);
		if (led_test_exit)
			break;
		sleep(1);
	} while (1);

	
	// switch all leds off
	while(led_seq[index] != NULL)
		write_int(led_seq[index++], 0);
	
	led_thread_exit = true;
	led_thread_start = false;
	LOGD("%s: Exit\n", __FUNCTION__);
	pthread_exit(NULL);
}
	return NULL;
}
#endif

static void *update_keypadled_thread(void *priv)
{
	int index = 0;
	// no LED for test
	if (keypadled_seq[0] == NULL) {
	pthread_exit(NULL);
		return NULL;
	}

	LOGD("%s: Start\n", __FUNCTION__);
	keypadled_thread_start = true;
	keypadled_thread_exit = false;

	do {
		write_int(keypadled_seq[index], 255);
		if (keypadled_test_exit)
			break;
		sleep(1);
		if (keypadled_test_exit)
			break;
		write_int(keypadled_seq[index], 0);
		sleep(1);
		
		if (keypadled_seq[++index] == NULL)
			index = 0;
		
	} while (1);

	// switch all leds off
	while(keypadled_seq[index] != NULL)
		write_int(keypadled_seq[index++], 0);
	
	keypadled_thread_exit = true;
	keypadled_thread_start = false;
	LOGD("%s: Exit\n", __FUNCTION__);
	pthread_exit(NULL);

	return NULL;
}


int led_entry(struct ftm_param *param, void *priv)
{
	int chosen;
	bool exit = false;
	struct itemview *iv;
	text_t    title;
	struct ftm_module *mod = (struct ftm_module *)priv;
	static int nled_cnt = 0;
	static int keypadled_cnt = 0;

	LOGD("%s\n", __FUNCTION__);

#if defined(RGB_LED_ALONE_TEST)
	int i=0;
 do
 	{
		led_test_exit[i] = false;
		i++;
 	}while(i<MAX_LED);
 keypadled_test_exit = false;
	led_inx=1;
	led_inx_pre = 0;
#else
	led_test_exit = false;
	keypadled_test_exit = false;
#endif

	iv = ui_new_itemview();
	if (!iv) {
		LOGD("No memory");
		return -1;
	}

	init_text(&title, param->name, COLOR_YELLOW);
	iv->set_title(iv, &title);
	iv->set_items(iv, items, 0);
if(status == 0){
	do {
		chosen = iv->run(iv, &exit);
		switch (chosen) {
			#if defined(RGB_LED_ALONE_TEST)
			case ITEM_KEYPAD_LED_TEST:
				nled_cnt = 0;
				//led_inx=1;

				if(keypadled_cnt == 0)
				{
							if(led_thread_start[led_inx-1])
							{
								led_test_exit[led_inx-1] = true;
								while(!led_thread_exit[led_inx-1])
								{
									//msleep(500);
									LOGD("%s: sleep\n", __FUNCTION__);
									sleep(1);
								}
								led_test_exit[led_inx-1] = false;
							}
					
					pthread_create(&keypadled_thread, NULL, update_keypadled_thread, priv);
				}

				keypadled_cnt++;
				
				break;

			case ITEM_RLED_TEST:
				led_inx_pre = 1;
				goto NLED_TEST_LOOP;
				break;
			case ITEM_GLED_TEST:
				led_inx_pre = 2;
				goto NLED_TEST_LOOP;
				break;
			case ITEM_BLED_TEST:
				led_inx_pre = 3;
NLED_TEST_LOOP:
				keypadled_cnt = 0;
				
				if(nled_cnt != led_inx_pre)
				{
					if(keypadled_thread_start)
					{
						keypadled_test_exit = true;
						while(!keypadled_thread_exit)
						{
							//msleep(500);
							LOGD("%s: sleep\n", __FUNCTION__);
							sleep(1);
						}
						keypadled_test_exit = false;
					}

					if(led_thread_start[led_inx-1])
					{
						led_test_exit[led_inx-1] = true;
						while(!led_thread_exit[led_inx-1])
						{
							//msleep(500);
							LOGD("%s: sleep\n", __FUNCTION__);
							sleep(1);
						}
						led_test_exit[led_inx-1] = false;
					}
					
					led_inx = led_inx_pre;
				pthread_create(&led_thread[led_inx-1], NULL, update_RGBled_thread, priv);
				nled_cnt = led_inx;
				}		
				
				break;
			#else
			case ITEM_KEYPAD_LED_TEST:
				nled_cnt = 0;

				if(keypadled_cnt == 0)
				{
					if(led_thread_start)
					{
						led_test_exit = true;
						while(!led_thread_exit)
						{
							//msleep(500);
							LOGD("%s: sleep\n", __FUNCTION__);
							sleep(1);
						}
						led_test_exit = false;
					}
					
					pthread_create(&keypadled_thread, NULL, update_keypadled_thread, priv);
				}

				keypadled_cnt++;
				
				break;
				
			case ITEM_NLED_TEST:
				keypadled_cnt = 0;
				
				if(nled_cnt == 0)
				{
					if(keypadled_thread_start)
					{
						keypadled_test_exit = true;
						while(!keypadled_thread_exit)
						{
							//msleep(500);
							LOGD("%s: sleep\n", __FUNCTION__);
							sleep(1);
						}
						keypadled_test_exit = false;
					}
					
				pthread_create(&led_thread, NULL, update_led_thread, priv);
				}		
				
				nled_cnt++;
				
				break;
		#endif
				
		case ITEM_PASS:
			mod->test_result = FTM_TEST_PASS;
			exit = true;
			break;
		case ITEM_FAIL:
			mod->test_result = FTM_TEST_FAIL;
			exit = true;
			break;
		default:
			//exit = true;
			break;
		}
		
		if (exit) {
			#if defined(RGB_LED_ALONE_TEST)
			led_test_exit[led_inx-1] = true;
			#else
			led_test_exit = true;
			#endif
				keypadled_test_exit = true;
				nled_cnt = 0;
				keypadled_cnt = 0;
			break;
		}		
	} 
	while (1);
}
else if(status == 1)
{
	iv->start_menu(iv, 0);
	iv->redraw(iv);
	#ifdef CUST_HAVE_KEYPAD_LED
		nled_cnt = 0;
		#if defined(RGB_LED_ALONE_TEST)
		led_inx=1;
		#endif

				if(keypadled_cnt == 0)
				{
					#if defined(RGB_LED_ALONE_TEST)
					if(led_thread_start[led_inx-1])
					{
						led_test_exit[led_inx-1] = true;
						while(!led_thread_exit[led_inx-1])
						{
							//msleep(500);
							LOGD("%s: sleep\n", __FUNCTION__);
							sleep(1);
						}
						led_test_exit[led_inx-1] = false;
					}
					#else
					if(led_thread_start)
					{
						led_test_exit = true;
						while(!led_thread_exit)
						{
							//msleep(500);
							LOGD("%s: sleep\n", __FUNCTION__);
							sleep(1);
						}
						led_test_exit = false;
					}
					#endif
					
					pthread_create(&keypadled_thread, NULL, update_keypadled_thread, priv);
				}

			//	keypadled_cnt++;
	#endif
	
	#ifndef CUST_HAVE_NLEDS
	
			keypadled_cnt = 0;
					
					if(nled_cnt == 0)
					{
						if(keypadled_thread_start)
						{
							keypadled_test_exit = true;
							while(!keypadled_thread_exit)
							{
								//msleep(500);
								LOGD("%s: sleep\n", __FUNCTION__);
								sleep(1);
							}
							keypadled_test_exit = false;
						}
					#if defined(RGB_LED_ALONE_TEST)
					pthread_create(&led_thread[led_inx-1], NULL, update_RGBled_thread, priv);
					#else
					pthread_create(&led_thread, NULL, update_led_thread, priv);
					#endif
					}		
					
				//	nled_cnt++;
		
	#endif
}

	#if defined(RGB_LED_ALONE_TEST)
	int k=0;
	do
	{
		pthread_join(led_thread[k], NULL);
		k++;
	}while(k<MAX_LED);
	#else
	pthread_join(led_thread, NULL);
	#endif
	pthread_join(keypadled_thread, NULL);

	return 0;
}

int led_init(void)
{
	int index;
	int ret = 0;
	struct ftm_module *mod;

	LOGD("%s\n", __FUNCTION__);
	
	mod = ftm_alloc(ITEM_LED, sizeof(struct ftm_module));
	if (!mod)
		return -ENOMEM;

	// switch all leds off
	while(led_seq[index] != NULL)
		write_int(led_seq[index++], 0);

	ret = ftm_register(mod, led_entry, (void*)mod);

	return ret;
}

#endif // FEATURE_FTM_LED
