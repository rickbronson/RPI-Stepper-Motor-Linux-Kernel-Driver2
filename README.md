  Raspberry PI 4 Stepper Motor Linux Kernel Driver Version 2
==========================================

NOTE: This is very much like the version one driver except it leaves the ramping up/down to you.  It's intended that you call the driver at a pretty high rate (60 - 250Hz) and you will be doing all the fine control over ramping up and down.  When you call down to the driver and set the speed, it sets the speed to that value and does not attempt to ramp up or down over the distance you specify.  You need to specify the distance only in so far as to 1) make sure you don't set it too long such that the build time exceeds the period that you are calling into the dirver and 2) specify it long enough such that it doesn't finish the distance before you call into the driver with your next command.

This is everything you need to build a stepper motor driver for the Raspberry PI 4 that is a real Linux kernel module.

This code consists of a Makefile, one driver C file, one test C file and a common shared header file

Features

- Driver runs in kernel and not in user space.
- Uses PWM to provide delays between motor STEP cycles
- Uses DMA to stream pulses
- Only uses 6 DMA's per motor STEP cycle
- Pulse edge granularity based on 27 MHz
- Can use any unused GPIO's for STEP, DIRECTION, and MICROSTEP control pins
- Driver smoothly ramps motor up to max speed and back down again based on a (approximate) sinusoidal curve
- Drive multiple motors.
- Can handle the same microstep control lines going to multiple motors, just use the same GPIO values as the first motor.

Constraints

- GPIO edge granularity is 1/PWM_FREQ, about 37ns
- Minimum time between any two GPIO edges (can be different GPIO's or the same) is about 500ns, this seems to be a constraint of the DMA
- Driver requires a minimum amount of steps when any motor is already running. If the DMA is stilll streaming to motor 1 and you ask to move motor 2, The driver has to copy existing DMA control blocks (CB's), build the new motors DMA CB's and then combine them into one stream in the existing live DMA buffer.  If the minimum is not met, I just wait until the existing stream is done.  The minimum is based on time it takes to do the copy and build plus a guess as to the time it takes to do the combine.  The fudge factor to estimate the combine time can be set via priv->step_cmd.combine_ticks_per_step.  NOTE: If the existing DMA buffer is appended with new streams (this only happens when the current DMA is not finished) enough times you will run into the end of the DMA buffer.
- When the minimum time cannot be satisfied then the driver will wait (or not) for the current DMA stream to finish for priv->step_cmd.wait_timeout (in milliseconds).  If this value is zero then it doesn't wait.
- Maximum number of steps is hardcoded as MAX_STEPS in rpi4-stepper.h.  This has a big impact on the amount of alloc'd memory.
- Max number of motors set by MAX_MOTORS.  This has a big impact on the amount of alloc'd memory.
- You may have to tweak DMA_CHANNEL depending on your kernel
- Minimum speed needs to be greater than 0

Untested
- Only tested on RPI-4

1. Get and build everything. We have to tweak the device tree and need a header inside the kernel so we have to build just the device tree.  Optionally, you can keep your old kernel and device tree so you can go back to that if need be.

```
git clone --recursive https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver.git
```

  At this point we need to get a kernel running that we can get the linux-headers for.  I couldn't not get the headers for the kernel I got with the zip file.

```
sudo cp -a /boot /boot.sav  # save old kernel and device tree (optional)
sudo apt update
sudo rpi-update stable  # get stable kernel
sudo reboot # after which you should be running new kernel
```
  Now get the kernel headers for the updated kernel and other programs we need to build the device tree.

```
sudo apt install raspberrypi-kernel-headers git bc bison flex libssl-dev make
git clone --depth=1 https://github.com/raspberrypi/linux
cd linux
make bcm2711_defconfig

```

  Now we need to tweaked a device tree file (NOTE: Only do this once!)

```
cat >> arch/arm/boot/dts/bcm270x.dtsi <<'EOF'
&pwm {                                                                        
      dmas = <&dma 5>;                                                        
      dma-names = "rx-tx";                                                    
      status = "okay";                                                        
};
EOF
```

  Then continue building the kernel device tree

```
make -j4 dtbs
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo reboot # to load the new device tree
```

  Lastly build this driver using the kernel headers installed above.

```
cd ../RPI-Stepper-Motor-Linux-Kernel-Driver/src/
make
```

2. Hookup
--------------

Simplified hookup of the motor with a DRV8825 driver board and serial debug
![Motor hookup](https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver/blob/master/docs/hardware/schematic10.png "Motor hookup")

3. Run
--------------

  do "./test-stepper --help" for test program options

```
cd ~/RPI-Stepper-Motor-Linux-Kernel-Driver/src
sudo rmmod pwm-bcm2835  # remove existing driver if any
sudo rmmod pwm-stepper-bcm2835 # remove our driver if it's already installed
sudo insmod ./pwm-stepper-bcm2835.ko
sudo test-stepper -d 500 -s 4000 -r 1 -m 7  # move 500 microsteps, start at 400Hz, max 4000Hz
sudo test-stepper -d 500 -s 4000 -r 3 -m 7  # same, more aggressive curve
```

  I tested my motor up to 215000 Hz (without load) at "-m 7" and it worked well.

4. Debug

  See pwm-stepper-bcm2835.c for printk statements.  Use "sudo dmesg" to view them.

  You can generate the speed profile using the test-stepper program with the "-g" option as below:

```
./test-stepper -d 256 -s 128000 -m 7 -g
```

5. Comments/suggestions

  Please contact me at rick AT efn DOT org
