ifneq ($(TARGET_SIMULATOR),true)
ifeq ($(TARGET_ARCH),arm)

LOCAL_PATH := $(call my-dir)
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/inc

#====================================================	
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	src/iap_8sec_ack_aa.c
LOCAL_MODULE:=elan_iap

include $(BUILD_EXECUTABLE)


#====================================================
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	src/ft5336_upgrade.c
LOCAL_MODULE:=ft5336_upgrade

include $(BUILD_EXECUTABLE)

endif   # TARGET_ARCH == arm
endif	# !TARGET_SIMULATOR
