#!/bin/sh
cd src
make ARCH=arm CROSS_COMPILE=/opt/tools/gcc5.3/arm-linux-gnueabihf/bin/arm-linux-gnueabihf- tisdk_am57xx-evm_defconfig 
make ARCH=arm CROSS_COMPILE=/opt/tools/gcc5.3/arm-linux-gnueabihf/bin/arm-linux-gnueabihf- menuconfig 
make ARCH=arm CROSS_COMPILE=/opt/tools/gcc5.3/arm-linux-gnueabihf/bin/arm-linux-gnueabihf- am57xx-evm-reva3.dtb 

