LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	backup.cpp

LOCAL_C_INCLUDES += \
	$(MTK_PATH_SOURCE)/external/nvram/libnvram \
	$(MTK_PATH_SOURCE)/external/nvram/libfile_op \
	$(MTK_PATH_CUSTOM)/cgen/inc	


LOCAL_SHARED_LIBRARIES := \
	libutils \
	libcutils \
	libbinder \
	libnvram \
	libfile_op \
	libcustom_nvram 
#LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_TAGS := optional


LOCAL_MODULE:= nvram_backup_binder

include $(BUILD_EXECUTABLE)

