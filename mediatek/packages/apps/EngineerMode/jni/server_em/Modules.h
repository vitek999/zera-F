/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
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

#ifndef __AFMMODULES__
#define __AFMMODULES__


#define FUNCTION_EM_BASEBAND	10001
#define FUNCTION_EM_CPU_FREQ_TEST_START	20001
#define FUNCTION_EM_CPU_FREQ_TEST_STOP	20002
#define FUNCTION_EM_CPU_FREQ_TEST_CURRENCT	20003
#define FUNCTION_EM_FB0_IOCTL	30001
#define FUNCTION_EM_CPUSTRESS_APMCU 40001
#define FUNCTION_EM_CPUSTRESS_SWCODEC 40002
#define FUNCTION_EM_CPUSTRESS_BACKUP 40003
#define FUNCTION_EM_CPUSTRESS_THERMAL 40004
#define FUNCTION_EM_SENSOR_DO_CALIBRATION 50001
#define FUNCTION_EM_SENSOR_CLEAR_CALIBRATION 50002
#define FUNCTION_EM_SENSOR_SET_THRESHOLD 50003
#define FUNCTION_EM_SENSOR_DO_GSENSOR_CALIBRATION 50004
#define FUNCTION_EM_SENSOR_GET_GSENSOR_CALIBRATION 50005
#define FUNCTION_EM_SENSOR_CLEAR_GSENSOR_CALIBRATION 50006
#define FUNCTION_EM_SENSOR_DO_GYROSCOPE_CALIBRATION 50007
#define FUNCTION_EM_SENSOR_GET_GYROSCOPE_CALIBRATION 50008
#define FUNCTION_EM_SENSOR_CLEAR_GYROSCOPE_CALIBRATION 50009
#define FUNCTION_EM_DFO_INIT 60001
#define FUNCTION_EM_DFO_READ_COUNT 60002
#define FUNCTION_EM_DFO_READ 60003
#define FUNCTION_EM_DFO_WRITE 60004
#define FUNCTION_EM_DFO_DEINIT 60005
#define FUNCTION_EM_DFO_PROPERTY_SET 60006
#define FUNCTION_EM_DFO_GET_DEFAULT_SIZE 60007
#define FUNCTION_EM_MSDC_SET_CURRENT 70001
#define FUNCTION_EM_MSDC_GET_CURRENT 70002
#define FUNCTION_EM_MSDC_SET30_MODE 70003
#define CKT_FUNCTION_EM_SENSOR_BASE  150000
#define CKT_FUNCTION_EM_SENSOR_GET_PS_DATA  (CKT_FUNCTION_EM_SENSOR_BASE+1)
#define CKT_FUNCTION_EM_SENSOR_GET_PS_Off_DATA (CKT_FUNCTION_EM_SENSOR_BASE+2)
#define CKT_FUNCTION_EM_SENSOR_DO_PS_CALIBRATION (CKT_FUNCTION_EM_SENSOR_BASE+3)
#define CKT_FUNCTION_EM_SENSOR_CLEAR_PS_CALIBRATION (CKT_FUNCTION_EM_SENSOR_BASE+4)
class RPCClient;

class AFMModules
{
public:
	AFMModules();
	virtual ~AFMModules();
	static void Execute(int feature_id, RPCClient* msgSender);
	//static void NotifyStop(int feature_id);
};

#endif	
