# Copyright (C) 2011 The Android-x86 Open Source Project

ifeq ($(strip $(BOARD_HAS_GPS_HARDWARE)),true)
LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked and stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SHARED_LIBRARIES := liblog libcutils
LOCAL_MODULE := gps.$(TARGET_PRODUCT)
LOCAL_MODULE_TAGS := optional
LOCAL_SRC_FILES := gps.c

ifeq ($(wildcard $(LOCAL_PATH)/power-$(TARGET_PRODUCT).c),)
LOCAL_SRC_FILES += power-stub.c
else
LOCAL_SRC_FILES += power-$(TARGET_PRODUCT).c
endif

include $(BUILD_SHARED_LIBRARY)
endif
