obj-m := pwm-sunxi.o
# KDIR := /lib/modules/$(shell uname -r)/build
KDIR := ~/src/linux-trees/build-boot-root/linux-sunxi/sun4i
PWD := $(shell pwd)
O=sun4i
ARCH=arm
CROSS_COMPILE=arm-linux-gnu-
all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean
