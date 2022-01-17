# Makefile for kernel module and test program

all:	driver test-stepper

obj-m += pwm-stepper-bcm2835.o

driver:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) EXTRA_CFLAGS+=-Wno-error=date-time modules
	objdump -DS $(obj-m) > $(obj-m).lis

clean:
	rm -f *.o test-stepper *.lis pwm-stepper*.lis
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

test-stepper:	test-stepper.c rpi4-stepper.h
	gcc -Wa,-alh,-L -c $@.c -o $@.o > $@.lis
	gcc -o $@ $@.o

mpu6050:	mpu6050.c
	gcc -ggdb -g -Wa,-alh,-L -c $@.c -o $@.o > $@.lis
	gcc -lX11 -lpthread -o $@ $@.o

test:	test.c
	gcc -ggdb -g -Wa,-alh,-L -c $@.c -o $@.o > $@.lis
	gcc -o $@ $@.o
