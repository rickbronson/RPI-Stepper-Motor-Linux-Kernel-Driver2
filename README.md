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
- Drive multiple motors.
- Can handle the same microstep control lines going to multiple motors, just use the same GPIO values as the first motor.

Constraints

- GPIO edge granularity is 1/PWM_FREQ, about 37ns
- Minimum time between any two GPIO edges (can be different GPIO's or the same) is about 500ns, this seems to be a constraint of the DMA
- Maximum number of steps is hardcoded as MAX_STEPS in rpi4-stepper.h.  This has a big impact on the amount of alloc'd memory.
- Max number of motors set by MAX_MOTORS.  This has a big impact on the amount of alloc'd memory.
- You may have to tweak DMA_CHANNEL depending on your kernel
- Minimum speed needs to be greater than 0

Untested
- Only tested on RPI-4

1. Get and build everything. We have to tweak the device tree and need a header inside the kernel so we have to build just the device tree.  Optionally, you can keep your old kernel and device tree so you can go back to that if need be.

```
git clone --recursive https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver2.git
```

  At this point we need to get a kernel running that we can get the linux-headers for.  I couldn't not get the headers for the kernel I got with the zip file.

```
sudo cp -a /boot /boot.sav  # save old kernel and device tree (optional)
sudo apt update
sudo apt install git bc bison flex libssl-dev make emacs xterm \
linux-headers raspberrypi-kernel-headers build-essential bc \
git wget bison flex libssl-dev make libncurses-dev

git clone --depth=1 https://github.com/raspberrypi/linux
cd linux
KERNEL=kernel7l-stepper
make bcm2711_defconfig
# config.txt file to select the kernel that the Pi will boot into: kernel=kernel-$USER.img
sed -i -e "s/CONFIG_LOCALVERSION=.*/CONFIG_LOCALVERSION=\"-v7l-stepper\"/" .config

make -j4 zImage modules dtbs
sudo make modules_install
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm/boot/zImage /boot/$KERNEL.img
sudo bash -c "echo kernel=$KERNEL.img >> /boot/config.txt"
sudo reboot # to check that the above all worked and you are running a new kernel, after reboot do:

uname -a  # should show a new kernel

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
make -j4 dtbs
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo reboot
```

  Lastly build this driver using the kernel headers installed above.

```
cd ../RPI-Stepper-Motor-Linux-Kernel-Driver2/src/
make
```

2. Hookup
--------------

Simplified hookup of the motor with a DRV8825 driver board and serial debug
![Motor hookup](https://github.com/rickbronson/RPI-Stepper-Motor-Linux-Kernel-Driver2/blob/master/docs/hardware/schematic12.png "Motor hookup")

3. Run
--------------

  do "./test-stepper --help" for test program options

```
cd ~/RPI-Stepper-Motor-Linux-Kernel-Driver2/src
sudo rmmod pwm-bcm2835  # remove existing driver if any
sudo rmmod pwm-stepper-bcm2835 # remove our driver if it's already installed
sudo insmod ./pwm-stepper-bcm2835.ko
sudo test-stepper -d 500 -s 4000 -m 7  # move 500 microsteps
```

  I tested my motor up to 215000 Hz (without load) at "-m 7" and it worked well.

4. Debug

  See pwm-stepper-bcm2835.c for printk statements.  Use "sudo dmesg" to view them.

5. Comments/suggestions

  Please contact me at rick AT efn DOT org
