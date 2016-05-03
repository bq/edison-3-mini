# This makefile is included from vendor/intel/common/wifi/WifiRules.mk
$(eval $(call build_kernel_module,$(call my-dir)/,rtl8189es,CONFIG_RTL8188E=m CONFIG_BT_COEXIST=n))
