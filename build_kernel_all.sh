#!/bin/sh
export KERNELDIR=`readlink -f .`

cd $KERNELDIR;

mkdir -p $KERNELDIR/../BUILT_OUTPUTS
chmod 777 $KERNELDIR/../BUILT_OUTPUTS
rm -rf $KERNELDIR/../BUILT_OUTPUTS/*
chmod 777 $KERNELDIR/build_kernel_*

echo "Building merlin .....";
cd $KERNELDIR/
./build_kernel_merlin.sh;
mv BUILT_*/ ../BUILT_OUTPUTS/
rm $KERNELDIR/.config

echo "Building osprey .....";
cd $KERNELDIR/
./build_kernel_osprey.sh;
mv BUILT_*/ ../BUILT_OUTPUTS/

