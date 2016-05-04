#!/bin/bash

echo -e "###### Cleaning old compilation ######\n"
rm -rf out

if [ ! -d x86_64-linux-android-4.8 ]; then
echo -e "###### Downloading gcc toolchain ######\n"
git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/x86/x86_64-linux-android-4.8 -b lollipop-release
fi

echo -e "###### Compiling kernel ######\n"
make -f Makefile modules_install
make -f Makefile rtl8723bs_install

echo -e "###### Copying modules ######\n"
cd out/target/product/bq_edison3_mini/linux/modules_install/lib/modules/3.10.20*
mkdir ../../../../../modules ; find . -name \*.ko -exec cp -pv {} ../../../../../modules \;

exit 0
