# Copyright (C) 2011 The Android-x86 Open Source Project

ifeq ($(strip $(BOARD_HAS_GPS_HARDWARE)),true)
LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked and stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils
LOCAL_MODULE := gps.default
LOCAL_MODULE_TAGS := optional
ifeq ($(strip $(BOARD_GPS_HARDWARE)),huawei)
LOCAL_SRC_FILES := gps_huawei.c
else
LOCAL_SRC_FILES := gps.c
endif
include $(BUILD_SHARED_LIBRARY)
endif
