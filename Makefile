################################################################################
#
# Common Variables that already set:
#     LICHEE_KDIR
#     LICHEE_MOD_DIR
#     CROSS_COMPILE
#     ARCH
#
#################################################################################
ARCH:= arm
CROSS_COMPILE:= /usr/bin/arm-linux-gnueabihf-
PWD=$(shell pwd)
KDIR:= $(PWD)/../linux-sunxi


all: build

obj-m+=lirc_send_pwm.o
#install: build
#	cp spi-sunxi-slave.ko $(LICHEE_MOD_DIR)/

build:
	@echo $(LICHEE_KDIR)
#	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(LICHEE_KDIR) M=$(PWD) modules
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD)

clean:
	@rm -rf *.o *.ko .*.cmd *.mod.c *.order *.symvers .tmp_versions *~
