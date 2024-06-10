
all: integrated_battery battery_update_linux
	
obj-m += integrated_battery.o

KERN_VER=$(shell uname -r)

integrated_battery: integrated_battery.c
	make -C /lib/modules/$(KERN_VER)/build M=$(shell pwd) modules

battery_update_linux: battery_update_linux.cpp adafruit_lc709203f_linux.cpp
	g++ -Wall battery_update_linux.cpp adafruit_lc709203f_linux.cpp -o battery_update_linux

clean:
	rm -f *.cmd *.ko *.o .Module.* .modules.* Module.* modules.order *.mod* .integrated_battery.* 
	rm -f battery_update_linux
