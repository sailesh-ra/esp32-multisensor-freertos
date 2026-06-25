# Makefile for sensor_ldisc kernel module
# Run on the Raspberry Pi (not cross-compiled):
#	make 		-- build sensor_ldisc.ko
#	make clean	-- remove build artifacts
#	make load	-- insmod + show dmesg tail
#	make unload	-- rmmod
#	make attach	-- attach ldisc	to /dev/ttyAMA0 (runs ldattach in background)
#	make detach	-- kill ldattach process

obj-m			:= sensor_ldisc.o
KDIR			:= /lib/modules/$(shell uname -r)/build
PWD				:= $(shell pwd)
EXTRA_CFLAGS	:= -I/home/sailesh/sensor_ldisc/include

# Module parameter overrides (used by make attach)
LDISC_NUM		?= 28
PROTO			?= 0
BAUD			?= 115200
TTY_DEV			?= /dev/ttyAMA0

all:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

load: all
	sudo insmod sensor_ldisc.ko ldisc_num=$(LDISC_NUM) proto=$(PROTO)
	@echo "--- dmesg tail ---"
	@dmesg | tail -5

unload:
	sudo rmmod sensor_ldisc || true
	@echo "---dmesg tail ---"
	@dmesg | tail -3

attach:
	sudo ldattach -8 -s $(BAUD) $(LDISC_NUM) $(TTY_DEV) &
	@echo "ldattach PID=$$!"
	@echo "ldisc $(LDISC_NUM) attached to $(TTY_DEV)"

detach:
	sudo killall ldattach 2>/dev/null && echo "ldattach stopped" || echo "ldattach not running"

# Full cycle: build -> load -> attach
start: load attach
	@echo "Ready: cat /dev/sensor_data"

stop: detach unload

.PHONY: all clean load unload attach detach start stop
