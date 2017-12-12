MODSRC=`pwd`
obj-m+=virt-snd-drv.o

PREFIX=/media/vivanov/DATA/PROJECTS_3/Jarvis/smartNode/rpi/code/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-

all:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C /media/vivanov/DATA/PROJECTS_3/Jarvis/smartNode/rpi/code/linux M=$(MODSRC) modules

clean:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C /media/vivanov/DATA/PROJECTS_3/Jarvis/smartNode/rpi/code/linux M=$(MODSRC) clean

