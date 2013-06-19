obj-m := pwm-sunxi.o
# KDIR := /lib/modules/$(shell uname -r)/build
KDIR := ~/src/linux-trees/bbr2/linux-sunxi/sun4i
PWD := $(shell pwd)
O=sun4i
ARCH=arm
CROSS_COMPILE=arm-linux-gnu-
all:
	make -C $(KDIR) M=$(PWD) O=$(O) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	make -C $(KDIR) M=$(PWD) O=$(O) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

