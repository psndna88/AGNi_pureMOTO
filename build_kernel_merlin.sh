#!/bin/sh
export KERNELDIR=`readlink -f .`
. ~/AGNi_stamp_MOTO.sh
#. ~/gcc-linaro-5.3-2016.02_arm-gnueabi.sh
#. ~/gcc-5.3-uber_arm-eabi.sh
. ~/gcc-6.0-uber_arm-eabi.sh
export ARCH=arm
if [ ! -f $KERNELDIR/.config ];
then
  make defconfig agni_merlin_defconfig
fi
. $KERNELDIR/.config
mv .git .git-halt
echo "Clearing DTB files ..."
cd $KERNELDIR/arch/arm/boot/dts/
rm msm8939-merlin-p0.dtb msm8939-merlin-p1.dtb msm8939-merlin-p1v3.dtb
rm msm8916-osprey-p2b.dtb msm8916-osprey-p2bd.dtb
echo "Cross-compiling kernel ..."
cd $KERNELDIR/
make -j3 || exit 1
rm -rf $KERNELDIR/BUILT_merlin
mkdir -p $KERNELDIR/BUILT_merlin/lib/modules
find -name '*.ko' -exec mv -v {} $KERNELDIR/BUILT_merlin/lib/modules/ \;
echo ""
echo "Stripping unneeded symbols and debug info from module(s)..."
${CROSS_COMPILE}strip --strip-unneeded $KERNELDIR/BUILT_merlin/lib/modules/*
mkdir -p $KERNELDIR/BUILT_merlin/lib/modules/pronto
mv $KERNELDIR/BUILT_merlin/lib/modules/wlan.ko $KERNELDIR/BUILT_merlin/lib/modules/pronto/pronto_wlan.ko
mv $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/BUILT_merlin/
echo ""
echo "Compiling >> dtbtool <<  for device tree image ..."
cd $KERNELDIR/tools/dtbtool; make
echo ""
echo "Generating Device Tree image (dt.img) ..."
$KERNELDIR/tools/dtbtool/dtbtool -o $KERNELDIR/BUILT_merlin/dt.img -s 2048 -p $KERNELDIR/scripts/dtc/ $KERNELDIR/arch/arm/boot/dts/ && rm $KERNELDIR/tools/dtbtool/dtbtool
cd $KERNELDIR/; mv .git-halt .git
echo ""
echo "BUILT_merlin is ready."
