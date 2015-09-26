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
/////////////////////////////////////////////////////////////////////////////////////////
    MSG_ID_MMI_SS_MTLR_BEGIN_IND,           // 0x1
    MSG_ID_MMI_SS_MTLR_BEGIN_RES_REQ,       // 0x2
    MSG_ID_MMI_SS_MTLR_BEGIN_RES_RSP,       // 0x3
    MSG_ID_MMI_SS_AERQ_BEGIN_IND,           // 0x4
    MSG_ID_MMI_SS_AERQ_BEGIN_RES_REQ,       // 0x5
    MSG_ID_MMI_SS_AERQ_BEGIN_RES_RSP,       // 0x6
    MSG_ID_MMI_SS_AERP_BEGIN_REQ,           // 0x7
    MSG_ID_MMI_SS_AERP_BEGIN_RSP,           // 0x8
    MSG_ID_MMI_SS_AERP_END_REQ,             // 0x9
    MSG_ID_MMI_SS_AERP_END_RSP,             // 0xA
    MSG_ID_MMI_SS_AECL_BEGIN_IND,           // 0xB
    MSG_ID_MMI_SS_AECL_BEGIN_RES_REQ,       // 0xC
    MSG_ID_MMI_SS_AECL_BEGIN_RES_RSP,       // 0xD
    MSG_ID_MMI_SS_MOLR_BEGIN_REQ,           // 0xE
    MSG_ID_MMI_SS_MOLR_BEGIN_RSP,           // 0xF
    MSG_ID_MMI_SS_MOLR_END_REQ,             // 0x10
    MSG_ID_MMI_SS_MOLR_END_RSP,             // 0x11
    MSG_ID_MMI_AGPS_ENABLE_DISABLE_REQ,     // 0x12
    MSG_ID_MMI_AGPS_ENABLE_DISABLE_RSP,     // 0x13
    MSG_ID_MMI_AGPS_KEY_UPDATE_REQ,         // 0x14
    MSG_ID_MMI_AGPS_KEY_UPDATE_RSP,         // 0x15
    MSG_ID_MMI_AGPS_NEW_KEY_NEEDED_IND,     // 0x16
    MSG_ID_MMI_AGPS_RESET_POSITIONING_IND,  // 0x17
    MSG_ID_MMI_AGPS_CP_START_IND,           // 0x18
    MSG_ID_MMI_AGPS_CP_END_IND,             // 0x19
    MSG_ID_MMI_AGPS_CP_ABORT_REQ,           // 0x1A
    MSG_ID_MMI_AGPS_CP_ABORT_RSP,           // 0x1B
    MSG_ID_SP_L4C_NBR_CELL_INFO_REG_REQ,    // 0x1C   // We can not use MSG_ID_L4C_NBR_CELL_INFO_REG_REQ because it is already defined in Typedef.h
    MSG_ID_SP_L4C_NBR_CELL_INFO_REG_CNF,    // 0x1D
    MSG_ID_SP_L4C_NBR_CELL_INFO_DEREG_REQ,  // 0x1E
    MSG_ID_SP_L4C_NBR_CELL_INFO_DEREG_CNF,  // 0x1F
    MSG_ID_SP_L4C_NBR_CELL_INFO_IND,        // 0x20
    MSG_ID_AGPS_AUTO_TEST_IND,              // 0x21
    MSG_ID_AGPS_CP_UP_STATUS_IND,           // 0x22
    MSG_ID_AGPS_MOLR_START_IND,             // 0x23
    MSG_ID_AGPS_MOLR_STOP_IND,              // 0x24
    MSG_ID_AGPS_MTLR_RESPONSE_IND,          // 0x25

	/*BELOW IS ADDED BY GUANG YU*/
	MSG_ID_DHCP_MBCI_IP_CONFIG_REQ,
	MSG_ID_DHCP_MBCI_IP_CONFIG_RSP,
	MSG_ID_DHCP_MBCI_IP_CONFIG_IND,
	/*below is for GPS desense*/
	IPC_MSG_ID_L4C_RF_INFO_IND,
	IPC_MSG_ID_L4C_RF_INFO_REQ,

	
    MSG_ID_AGPS_CONTROL_PLANE_20_END,       


	
//////////////////////////////////////////////////////////////////////////////////////////


