DIR=`pwd`

BOARD="nrf52dk_nrf52832"
#BOARD="b_l072z_lrwan1"

BUILD_DIR=${DIR}/_BUILD_${BOARD}

#export GNUARMEMB_TOOLCHAIN_PATH=/usr/local/pkg/cross-arm-none-eabi
export GNUARMEMB_TOOLCHAIN_PATH=$HOME/tmp/at/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi

cd ~/src/zephyrproject/zephyr && \
	env \
		ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb \
		west -v build -d "$BUILD_DIR" -b $BOARD "$DIR"
		#west -v flash -d "$BUILD_DIR" --openocd ~/bin/openocd

