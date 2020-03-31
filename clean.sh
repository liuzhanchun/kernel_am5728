#!/bin/sh
cd src
make ARCH=arm CROSS_COMPILE=/opt/tools/gcc5.3/arm-linux-gnueabihf/bin/arm-linux-gnueabihf- distclean
