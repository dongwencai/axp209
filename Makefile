ARCH ?=mips
CROSS_COMPILE ?=mipsel-openwrt-linux-
CC=$(CROSS_COMPILE)gcc
obj-m := axp209.o                   #要生成的模块名    
mymodules-objs:= axp209.o        #生成这个模块名所需要的目标文件

KDIR :=/home/mrdong/openwrt/openwrt/build_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/linux-ramips_mt7620a/linux-3.10.49/ 

PWD := $(shell pwd)

default:
	$(CC) demo.c -o demo
	$(CC) axp.c -o axp209 -lm
	make -C $(KDIR) M=$(PWD) modules

clean:
	rm -rf *.o .* .cmd *.ko *.mod.c .tmp_versions demo axp209
