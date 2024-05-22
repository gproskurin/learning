DIR=`pwd`
#PRJ="${HOME}/src/zephyrproject/zephyr/samples/subsys/lorawan/class_a"
PRJ="${HOME}/src/zephyrproject/zephyr/samples/drivers/lora/receive"

#BOARD="nrf52dk_nrf52832"
#BOARD="b_l072z_lrwan1"
BOARD="nucleo_wl55jc"

BUILD_DIR=${DIR}/_BUILD_lr_${BOARD}

#export GNUARMEMB_TOOLCHAIN_PATH=/usr/local/pkg/cross-arm-none-eabi
export GNUARMEMB_TOOLCHAIN_PATH=$HOME/tmp/at/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi

cd ~/src/zephyrproject/zephyr && \
	env \
		ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb \
		west -v build -d "$BUILD_DIR" -b $BOARD $PRJ
		#west -v flash -d "$BUILD_DIR" --openocd ~/bin/openocd

