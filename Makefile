KERNEL_SRC_DIR := linux/kernel
TARGET_BOARD_PLATFORM := baytrail
PRODUCT_NAME := bq_edison3_mini
TARGET_DEVICE_DIR := device/intel/$(TARGET_BOARD_PLATFORM)/$(PRODUCT_NAME)
PRODUCT_OUT := out/target/product/$(PRODUCT_NAME)
BOARD_USE_64BIT_KERNEL := true
SUPPORT_3G_DONGLE_ONLY := true
KERNEL_DIFFCONFIG := $(TARGET_DEVICE_DIR)/$(PRODUCT_NAME)_64_diffconfig
KERNEL_DIFFCONFIG += device/intel/common/memory/zram_diffconfig
KERNEL_DIFFCONFIG += device/intel/common/memory/compaction_diffconfig
PATH := $(shell pwd)/x86_64-linux-android-4.7/bin:$(PATH)
TARGET_TOOLS_PREFIX := x86_64-linux-android-4.7/bin/x86_64-linux-android-
TARGET_CC := x86_64-linux-android-4.7/bin/x86_64-linux-android-gcc
MINIGZIP := bin/minigzip
openssl: bin/openssl

include device/intel/common/AndroidKernel.mk
include $(shell pwd)/linux/modules/wlan/realtek/AndroidRtl8723bs.mk
