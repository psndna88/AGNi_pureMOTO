#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/AGNi_stamp_MOTO.sh
#. ~/gcc-6.0-uber_arm-eabi.sh
. ~/gcc-6.x-uber_arm-eabi.sh
export ARCH=arm
if [ ! -f $KERNELDIR/.config ];
then
  make defconfig agni_osprey_defconfig
fi
. $KERNELDIR/.config
mv .git .git-halt
echo "Clearing DTB files ..."
rm $KERNELDIR/arch/arm/boot/dts/*.dtb
echo "Cross-compiling kernel ..."
cd $KERNELDIR/
make -j3 || exit 1
rm -rf $KERNELDIR/BUILT_osprey
mkdir -p $KERNELDIR/BUILT_osprey/lib/modules
find -name '*.ko' -exec mv -v {} $KERNELDIR/BUILT_osprey/lib/modules/ \;
echo ""
echo "Stripping unneeded symbols and debug info from module(s)..."
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT_osprey/lib/modules/*
mkdir -p $KERNELDIR/BUILT_osprey/lib/modules/pronto
mv $KERNELDIR/BUILT_osprey/lib/modules/wlan.ko $KERNELDIR/BUILT_osprey/lib/modules/pronto/pronto_wlan.ko
mv $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT_osprey/
echo ""
echo "Compiling >> dtbtool <<  for device tree image ..."
cd $KERNELDIR/tools/dtbtool; make
echo ""
echo "Generating Device Tree image (dt.img) ..."
$KERNELDIR/tools/dtbtool/dtbtool -o $KERNELDIR/BUILT_osprey/dt.img -s 2048 -p $KERNELDIR/scripts/dtc/ $KERNELDIR/arch/arm/boot/dts/ && rm $KERNELDIR/tools/dtbtool/dtbtool
cd $KERNELDIR/; mv .git-halt .git
echo ""
echo "BUILT_osprey is ready."
