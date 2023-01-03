GCC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

all: firmware.bin

clean:
	rm -f -- main.o firmware.elf firmware.bin

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $^ $@

firmware.elf: main.o linker.ld
	$(GCC) -T linker.ld -nostdlib main.o -o $@

main.o: main.c
	$(GCC) -mcpu=cortex-m3 $^ -c -O

