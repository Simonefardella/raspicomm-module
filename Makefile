obj-m += raspicommrs485.o

raspicommrs485-objs := module.o queue.o

RPICOMM_K_VERS=$(shell uname -r)
RPICOMM_MOD_DIR=/lib/modules/$(RPICOMM_K_VERS)
RPICOMM_BUILD=$(RPICOMM_MOD_DIR)/build
RPICOMM_INST=$(RPICOMM_MOD_DIR)/kernel/drivers/tty/serial
RPICOMM_VERSION=$(shell cat version.txt)
RPICOMM_DEBUG=$(shell awk '/pre[0-9]*$$/{print "-DDEBUG"}' < version.txt)
RPICOMM_RELEASE=binaries/$(RPICOMM_K_VERS)

FLAGS=-Werror -Wall $(RPICOMM_DEBUG) -DRASPICOMM_VERSION='\"$(RPICOMM_VERSION)\"'

# all: allmodules binaries/spi0devdis.dtbo
all: allmodules

allmodules:
	$(MAKE) -C $(RPICOMM_BUILD) M=$(PWD) KCPPFLAGS="$(FLAGS)" modules

clean:
	make -C $(RPICOMM_BUILD) M=$(PWD) clean

# install: all /boot/overlays/spi0devdis.dtbo $(RPICOMM_INST)/raspicommrs485.ko
install: all $(RPICOMM_INST)/raspicommrs485.ko
	if [ -z "$(DEBUG)" ]; then mkdir -p $(RPICOMM_RELEASE) && cp -pf raspicommrs485.ko $(RPICOMM_RELEASE)/ ; fi

$(RPICOMM_INST)/raspicommrs485.ko: raspicommrs485.ko
	sudo cp raspicommrs485.ko $(RPICOMM_INST)
	sudo depmod -a

binaries/spi0devdis.dtbo: spi0devdis.dts
	dtc -@ -I dts -O dtb -o binaries/spi0devdis.dtbo spi0devdis.dts

/boot/overlays/spi0devdis.dtbo: binaries/spi0devdis.dtbo
	sudo cp binaries/spi0devdis.dtbo /boot/overlays/spi0devdis.dtbo
	# echo "dtoverlay=spi0devdis" >> /boot/config.txt

