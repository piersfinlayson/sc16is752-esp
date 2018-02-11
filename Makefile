# Makefile for esp-sdk-samples
SDK_BASE ?= /opt/esp-open-sdk
SDK_BIN ?= $(SDK_BASE)/xtensa-lx106-elf/bin
PATH := $(SDK_BIN):$(PATH)
CC = $(SDK_BIN)/xtensa-lx106-elf-gcc
ESPTOOL ?= $(SDK_BIN)/esptool.py
CFLAGS = -Os -Iinclude -I./ -mlongcalls -DICACHE_FLASH
LDLIBS = -nostdlib -Wl,-static -Wl,--start-group -lc -lcirom -lgcc -lhal -lphy -lpp -lnet80211 -lwpa -lmain -llwip -Wl,--end-group
LDFLAGS = -Teagle.app.v6.ld

app_image-0x00000.bin: app_image
	$(ESPTOOL) elf2image $^

app_image: user_main.o pin_map.o brzo_i2c/brzo_i2c.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

# with latest chips need to specify dio for at least bootloader or csum err ensuesmak
flash: app_image-0x00000.bin
	$(ESPTOOL) --baud 115200 write_flash -fm dio -ff 40m -fs 32m 0x3fc000 $(SDK_BASE)/sdk/bin/esp_init_data_default.bin
	$(ESPTOOL) --baud 115200 write_flash -fm dio -ff 40m -fs 32m 0 app_image-0x00000.bin 0x10000 app_image-0x10000.bin

erase_flash:
	$(ESPTOOL) erase_flash

con74:
	miniterm.py /dev/ttyUSB0 74880 --raw

con:
	miniterm.py /dev/ttyUSB0 115200 --raw

con230:
	miniterm.py /dev/ttyUSB0 230400 --raw

clean: 
	@rm -f app_image* *.o brzo_i2c/*.o
