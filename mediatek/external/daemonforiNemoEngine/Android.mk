ifneq ($(TARGET_SIMULATOR),true)
ifeq ($(TARGET_ARCH),arm)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/inc

LOCAL_SRC_FILES := \
	src/main.c
	
LOCAL_MODULE:=daemon_iNemoEngine

LOCAL_LDFLAGS += $(LOCAL_PATH)/lib/iNemoEngine_SensorFusion.a
LOCAL_SHARED_LIBRARIES := liblog libcutils

include $(BUILD_EXECUTABLE)

endif   # TARGET_ARCH == arm
endif	# !TARGET_SIMULATOR



