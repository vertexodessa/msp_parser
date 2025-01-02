#!/bin/bash
DL="https://github.com/OpenIPC/firmware/releases/download/toolchain/toolchain"

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 [goke|hisi|star6b0|star6e|native]"
	exit 1
fi

if [[ "$1" == *"star6b0" ]]; then
	CXX=sigmastar-infinity6b0
elif [[ "$1" == *"star6e" ]]; then
	CXX=sigmastar-infinity6e
elif [[ "$1" == *"goke" ]]; then
	CXX=goke-gk7205v200
elif [[ "$1" == *"hisi" ]]; then
	CXX=hisilicon-hi3516ev200
fi

GCC=$PWD/toolchain/$CXX/bin/arm-linux-g++
OUT=msp_parser

if [[ "$1" != *"native"* && "$1" != *"rockhip"* ]]; then
	if [ ! -e toolchain/$CXX ]; then
		wget -c -q --show-progress $DL.$CXX.tgz -P $PWD
		mkdir -p toolchain/$CXX
		tar -xf toolchain.$CXX.tgz -C toolchain/$CXX --strip-components=1 || exit 1
		rm -f $CXX.tgz
	fi
	OUT=${OUT}_$1
fi

if [ ! -e firmware ]; then
	git clone https://github.com/openipc/firmware --depth=1
fi

if [ "$1" = "goke" ]; then
	DRV=$PWD/firmware/general/package/goke-osdrv-gk7205v200/files/lib
	make -B CXX=$GCC DRV=$DRV TOOLCHAIN=$PWD/toolchain/$CXX OUTPUT=$OUT $1
elif [ "$1" = "hisi" ]; then
	DRV=$PWD/firmware/general/package/hisilicon-osdrv-hi3516ev200/files/lib
	make -B CXX=$GCC DRV=$DRV TOOLCHAIN=$PWD/toolchain/$CXX OUTPUT=$OUT $1
elif [ "$1" = "star6b0" ]; then
	DRV=$PWD/firmware/general/package/sigmastar-osdrv-infinity6b0/files/lib
	make -B CXX=$GCC DRV=$DRV TOOLCHAIN=$PWD/toolchain/$CXX OUTPUT=$OUT $1
elif [ "$1" = "star6e" ]; then
	DRV=$PWD/firmware/general/package/sigmastar-osdrv-infinity6e/files/lib
	make -B CXX=$GCC DRV=$DRV TOOLCHAIN=$PWD/toolchain/$CXX OUTPUT=$OUT $1
elif [ "$1" = "rockchip" ]; then
    ./build_rockchip.sh $1
else
	DRV=$PWD
	make DRV=$DRV OUTPUT=$OUT $1
fi
