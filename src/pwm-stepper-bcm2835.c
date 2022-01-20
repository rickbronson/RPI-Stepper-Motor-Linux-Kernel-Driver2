/*
 * Stepper motor driver for Raspberry PI 4
 *
 * Copyright (C) 2022 Rick Bronson
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Send feedback to <rick@efn.org>
 *
 * Description : Stepper motor driver for Raspberry PI 4
 *
 * Based in some part upon drivers/char/tlclk.c (Copyright (C) 2005 Kontron Canada)
 * and drivers/pwm/pwm-bcm2835.c Copyright 2014 Bart Tanghe
 * dma based on drivers/mmc/host/bcm2835-mmc.c

To build, install headers:

sudo apt install raspberrypi-kernel-headers

NOTE: need to blacklist pwm-bcm2835
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>
#include <linux/errno.h>	/* error codes */
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_data/dma-bcm2708.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/of_address.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/of_dma.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/pwm.h>
#include "rpi4-stepper.h"

MODULE_AUTHOR("Rick Bronson <rick@efn.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom BCM2835 PWM for stepper motor driver");

#define RPI4_CRYSTAL_FREQ 54000000
#define PWM_FREQ (RPI4_CRYSTAL_FREQ / 2)  /* set for max granularity */

#define PERI_DMA_MASK 0x7fffffff  /* DMA seems to have a different view of addresses */
#define PERI_BASE   0xfe000000  /* 0xfe000000 for BCM2711,
																	 0x20000000 for BCM2835,
																	 0x3f000000 for BCM2836/7 */
#define GPIO_BASE (PERI_BASE + 0x00200000)  /* GPIO registers base address. */
#define PWM_BASE (PERI_BASE + 0x0020c000)  /* PWM registers base address */
#define PWM_CLK_BASE (PERI_BASE + 0x00101000)
#define DMA_BASE   (PERI_BASE + 0x00007000)
#define DMA15_BASE (PERI_BASE + 0x00e05000)
#define SYSTEM_TIMER_CLO (PERI_BASE + 0x00003004)  /* based on 1 MHz */
#define PWMCLK_CTL 40
#define PWMCLK_DIV 41

#define GPSET0 7  /* offset for gpio set/clr */
#define GPCLR0 10
#define GPIO_CLR (GPIO_BASE + GPCLR0 * 4)
#define GPIO_SET (GPIO_BASE + GPSET0 * 4)

#define PWM_RNG1     4  /* offsets for PWM reg's */
#define PWM_FIFO1    6

#define PWM_CTL_MSEN2 (1 << 15)
#define PWM_CTL_PWEN2 (1 << 8)
#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

#define PWM_DMAC_ENAB      (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ_VAL 10
#define PWM_DMAC_DREQ(x)   (x)

#define CLK_PASS 0x5a000000  /* clock password */
#define CLK_CTL_MASH(x) ((x) << 9)
#define CLK_CTL_BUSY    (1 << 7)
#define CLK_CTL_KILL    (1 << 5)
#define CLK_CTL_ENAB    (1 << 4)
#define CLK_CTL_SRC(x)  ((x) << 0)

#define TO_PHYS_KLUDGE(x) (__pa(x) | 0xc0000000)  // be nice to get rid of this

// #define DEBUG
#ifdef DEBUG
#define NOINLINE noinline
#define PRINTI(fmt, args...) printk(KERN_INFO fmt, ## args)
#define report_debug(num) \
	PRINTI("pwm-stepper debug " num " conblk = 0x%x cs = 0x%x build steps = %d\n", (int) priv->dma_regs->conblk_ad, (int) priv->dma_regs->cs, build_steps);
#else
#define NOINLINE
#define PRINTI(fmt, args...)
#define report_debug(num)
#endif	/* ATA_VERBOSE_DEBUG */

typedef enum {  /* for build_dma_thread() */
	USE_DMA_BUF,
	USE_BUILD_BUF,} BUILDTYPE;

typedef enum {  /* for waiting when busy */
	STEPPER_STATE_IDLE,
	STEPPER_STATE_BUSY,} STATETYPE;
	
/* GPIO registers */
struct S_GPIO_REGS
	{
	uint32_t gpfsel[6]; uint32_t reserved0;
	uint32_t gpset[2]; uint32_t reserved1;
	uint32_t gpclr[2]; uint32_t reserved2;
	uint32_t gplev[2]; uint32_t reserved3;
	uint32_t gpeds[2]; uint32_t reserved4;
	uint32_t gpren[2]; uint32_t reserved5;
	uint32_t gpfen[2]; uint32_t reserved6;
	uint32_t gphen[2]; uint32_t reserved7;
	uint32_t gplen[2]; uint32_t reserved8;
	uint32_t gparen[2]; uint32_t reserved9;
	uint32_t gpafen[2]; uint32_t reserved10;
	uint32_t gppud;
	uint32_t gppudclk[2]; uint32_t reserved11[4];
};

/* PWM reg's */
struct S_PWM_CTL {
	unsigned pwen1 : 1;
	unsigned mode1 : 1;
	unsigned rptl1 : 1;
	unsigned sbit1 : 1;
	unsigned pola1 : 1;
	unsigned usef1 : 1;
	unsigned clrf : 1;
	unsigned msen1 : 1;
	unsigned pwen2 : 1;
	unsigned mode2 : 1;
	unsigned rptl2 : 1;
	unsigned sbit2 : 1;
	unsigned pola2 : 1;
	unsigned usef2 : 1;
	unsigned reserved1 : 1;
	unsigned msen2 : 1;
	unsigned reserved2 : 16;
};

/* PWM registers */
struct S_PWM_REGS
	{
	union {
		uint32_t ctl;
		struct S_PWM_CTL ctl_bits;
		} control;

	uint32_t sta;
	uint32_t dmac;
#define DMA_ENAB (1 << 31)
	uint32_t reserved0;
	uint32_t rng1;
	uint32_t dat1;
	uint32_t fif1;
	uint32_t reserved1;
	uint32_t rng2;
	uint32_t dat2;
	};

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS          (1 << 26)
#define DMA_PERIPHERAL_MAPPING(x) ((x) << 16)
#define DMA_BURST_LENGTH(x)       ((x) << 12)
#define DMA_SRC_IGNORE              (1 << 11)
#define DMA_SRC_DREQ                (1 << 10)
#define DMA_SRC_WIDTH               (1 <<  9)
#define DMA_SRC_INC                 (1 <<  8)
#define DMA_DEST_IGNORE             (1 <<  7)
#define DMA_DEST_DREQ               (1 <<  6)
#define DMA_DEST_WIDTH              (1 <<  5)
#define DMA_DEST_INC                (1 <<  4)
#define DMA_WAIT_RESP               (1 <<  3)
#define DMA_TDMODE                  (1 <<  1)
#define DMA_INTEN                  (1 <<  0)

#define NORMAL_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP)
#define TIMED_DMA(x) (DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(x))

	struct S_DMA_REGS
	{
	uint32_t cs; /* DMA Control and Status */
/* DMA CS Control and Status bits */
#define DMA_CHANNEL_RESET       (1 << 31)
#define DMA_CHANNEL_ABORT       (1 << 30)
#define DMA_WAIT_ON_WRITES      (1 << 28)
#define DMA_PANIC_PRIORITY(x) ((x) << 20)
#define DMA_PRIORITY(x)       ((x) << 16)
#define DMA_INTERRUPT_STATUS    (1 <<  2)
#define DMA_END_FLAG            (1 <<  1)
#define DMA_ACTIVE              (1 <<  0)
	uint32_t conblk_ad; /* DMA Control Block Address */
	uint32_t pad1[6];
	uint32_t debug; /* DMA Channel Debug */
#define DMA_DEBUG_READ_ERR           (1 << 2)
#define DMA_DEBUG_FIFO_ERR           (1 << 1)
#define DMA_DEBUG_RD_LST_NOT_SET_ERR (1 << 0)
	uint32_t pad2[64 - 9];  /* pad out to 0x100 */
	};
/**
 * Private data
 */
struct dma_cb1 {  /* One DMA Control Block (CB), borrowed from include/linux/platform_data/dma-bcm2708.h */
	u32 info;
	u32 src;
	u32 dst;
	u32 length;
	u32 stride;
	u32 next;
	u16 index;  /* DMA unused, we use as index */
	u16 motor;  /* DMA unused, we use as motor */
	u32 range;  /* DMA unused, we use as range, NOTE: range is used in PWM sense, ie. it's a delay */
};

struct dma_cb3 {  /* to make it easier to deal with 3 control blocks that entail one time delay */
	u32 info1;
	u32 *src1;  /* pointer to range value */
	u32 dst1;  /* GPIO perif set/clr reg */
	u32 length1;
	u32 stride1;
	u32 next1;
#define STEP_INDEX_MOTOR 0  /* index in upper 16 bits, motor in lower */
#define STEP_RANGE 1
	u16 index;  /* DMA unused, we use as index */
	u16 motor;  /* DMA unused, we use as motor */
	u32 range;  /* DMA unused, we use as range */
	u32 info2;
	u32 *src2;  /* any valid pointer */
	u32 dst2;  /* PWM perif range reg */
	u32 length2;
	u32 stride2;
	u32 next2;
	u32 pad2[2];
	u32 info3;
	u32 src3;  /* pointer to GPIO value */
	u32 dst3;  /* PWM perif FIFO reg */
	u32 length3;
	u32 stride3;
	u32 next3;
	u32 pad3[2];
};

#define PWM_CBS_PER_STEP 6  /* PWM control blocks per step */
#define PWM_MAX_STEP_CBS (MAX_STEPS * PWM_CBS_PER_STEP)
struct pwm_dma_data {  /* everything got with alloc coherent */
	struct dma_cb1 run_cbs1[PWM_MAX_STEP_CBS * MAX_MOTORS];  /* DMA run cbs's, must be on 8 word boundry */
	struct dma_cb1 run_cbs2[PWM_MAX_STEP_CBS * MAX_MOTORS];  /* DMA run cbs's, must be on 8 word boundry */
	u32 range[MAX_STEPS * 2];  /* * 2 because we won't always have the same delay for each half cycle */
	u32 gpio_set_mask[MAX_MOTORS];  /* mask for setting microstep/dir GPIO's */
	u32 gpio_clr_mask[MAX_MOTORS];  /* mask for clearing microstep/dir GPIO's */
	};

struct stepper_priv {
	struct completion dma_cmpl;
	struct STEPPER_SETUP step_cmd[MAX_MOTORS];
	struct STEPPER_SETUP step_cmd_read;
	volatile unsigned int *pwm_clk_regs; /* Holds the address of PWM CLK registers */
	volatile struct S_DMA_REGS *dma_regs;
#define BCM2711_DMA_CHANNELS   15  /* don't use chan 16 */
	volatile unsigned int *system_timer_regs;
	struct platform_device *pdev;
	struct S_PWM_REGS *pwm_regs;
	struct S_GPIO_REGS *gpio_regs;
	spinlock_t lock;
	struct dma_chan	*dma_chan_tx;  /* DMA channel for writes */
	int irq_number;
	dma_addr_t dma_handle;
	struct pwm_dma_data *dma_send_buf;  /* dma data to send to motor step pin via pwm */
	int dma_size;
	struct clk *clk;
	int motors;  /* number of motors */
	struct dma_cb1 build_cbs1[PWM_MAX_STEP_CBS * MAX_MOTORS];  /* places to build the Control Blocks from new command */
	struct dma_cb1 build_cbs2[PWM_MAX_STEP_CBS * MAX_MOTORS];
#if MAX_MOTORS > 2
	struct dma_cb1 build_cbs3[PWM_MAX_STEP_CBS * MAX_MOTORS];
#endif
	struct dma_cb1 *current_dma_buf;  /* current DMA pointer to one of run_cbs1 or run_cbs2 */
#define GPIO_RPI4_MAX 28
	unsigned char gpio_requested[GPIO_RPI4_MAX];  /* marks if these gpio's are already requested */
#define NO_MOTOR 255
	unsigned char gpio2motor[GPIO_RPI4_MAX];  /* Keeps track of motor index */
	int timer_save;
	STATETYPE state;
	wait_queue_head_t wait_q;
	};

/* request one GPIO output, set to value */
static int request_set_gpio(struct stepper_priv *priv, GPIO pin, int value)
	{
	int ret = 0;

	if (!priv->gpio_requested[pin]) {  /* only request if we don't already have it */
		ret = gpio_request(pin, NULL);
		if (ret)
			printk(KERN_ERR "pwm-stepper Can't request gpio pin %d\n", pin);
		else {
			gpio_direction_output(pin, value & 1);
			priv->gpio_requested[pin] = 1;
			}
		}
	return ret;
	}

/* fast copy 3 CB's, way faster than memcpy */
static NOINLINE __naked void memcpy_cb3(struct dma_cb3 *dst, struct dma_cb3 *src, int size)
	{
	asm volatile (
    "    mov	ip, sp     \n"
    "    push	{r3-r9, ip, lr, pc}\n"
		"    ldmia  r1!, {r2-r9}     \n"
		"    stmia  r0!, {r2-r9}     \n"
		"    ldmia  r1!, {r2-r9}     \n"
		"    stmia  r0!, {r2-r9}     \n"
		"    ldmia  r1!, {r2-r9}     \n"
		"    stmia  r0!, {r2-r9}     \n"
    "    ldm	sp, {r3-r9, sp, pc}   \n"); 
	}

#define MIN_RANGE_TIME 23  /* in 1 / PWM_FREQ units, 22 or below causes whole step at this period */
#define CB_RANGE_OFFSET 0
#define range_addr src1  /* this get's set according to CB_RANGE_OFFSET above ie, 0 is src1, 1 is src2, etc */
/* combine from priv->build_cbs and pCbs_src to run_cbs, return number of blocks combined, takes about 1800us for total of 800+214 steps */
static NOINLINE int combine_dma_threads(struct stepper_priv *priv, struct dma_cb3 *pCbs_dest, struct dma_cb3 *pCbs_src1, struct dma_cb3 *pCbs_src2, BUILDTYPE flag)
	{
	struct pwm_dma_data *dma_send_buf = priv->dma_send_buf;
	u32 *p_range;
	u32 range, next1 = (u32) pCbs_src1, next2 = (u32) pCbs_src2;  /* NOTE: either could be NULL */
	int total1 = 0, total2 = 0, half_steps = 0, half_step_offset = 0;
	struct dma_cb3 *pCbs_end = (struct dma_cb3 *) (((struct dma_cb1 *) pCbs_dest) + ARRAY_SIZE(priv->build_cbs1));
	
//	printk(KERN_ERR "pwm-stepper combine_dma_threads pCbs_dest = 0x%x, *pCbs_src1 = 0x%x, *pCbs_src2 = 0x%x, half_step_offset = %d, flag = %d\n",
//		(int) pCbs_dest, (int) pCbs_src1, (int) pCbs_src2, half_step_offset, flag);
	while (next1 || next2) {  /* while either thread is active */
		if (next1 && (total2 >= total1 || !next2)) {  /* do thread1 */
			next1 = pCbs_src1->next3;  /* next pointer in linked list */
			range = pCbs_src1->range;  /* save range */
			if ((next2 && total1 + range > total2) || !next1) {
				if (total2 >= total1)
					pCbs_src1->range = max(total2 - total1, MIN_RANGE_TIME);  /* set for minimum if needed */
				}
			total1 += range;  /* keep track of total for thread1 */
			memcpy_cb3(pCbs_dest, pCbs_src1, sizeof (*pCbs_dest));  /* do thread1 */
			p_range = &dma_send_buf->range[half_step_offset];
			pCbs_dest->range_addr = p_range;  /* set range addr and range */
			if (flag == USE_DMA_BUF)
				*p_range = pCbs_src1->range;
			pCbs_src1++;  /* move to next step */
			pCbs_dest->next1 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 1);
			pCbs_dest->next2 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 2);
			pCbs_dest->next3 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 3);
			pCbs_dest->info3 &= ~DMA_INTEN;  /* remove interupt on this one */
			half_steps++;
			half_step_offset++;
			pCbs_dest++;
			}
		else
			if (next2 && (total1 > total2 || !next1)) {  /* do thread2 */
				next2 = pCbs_src2->next3;
				range = pCbs_src2->range;
				if ((next1 && total2 + range > total1) || !next2) {
					if (total1 > total2)
						pCbs_src2->range = max(total1 - total2, MIN_RANGE_TIME);  /* set for minimum if needed */
					}
				total2 += range;
				memcpy_cb3(pCbs_dest, pCbs_src2, sizeof (*pCbs_dest));
				p_range = &dma_send_buf->range[half_step_offset];
				pCbs_dest->range_addr = p_range;  /* set range addr and range */
				if (flag == USE_DMA_BUF)
					*p_range = pCbs_src2->range;
				pCbs_src2++;  /* move to next step */
				pCbs_dest->next1 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 1);
				pCbs_dest->next2 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 2);
				pCbs_dest->next3 = TO_PHYS_KLUDGE((struct dma_cb1 *) pCbs_dest + 3);
				pCbs_dest->info3 &= ~DMA_INTEN;  /* remove interupt on this one */
				half_steps++;
				half_step_offset++;
				pCbs_dest++;
				}
		if (pCbs_dest >= pCbs_end) { /* check for end */
			printk(KERN_ERR "pwm-stepper Exceeding size of max steps, pCbs_dest=0x%x dma_buf_end=0x%x\n",
				(int) pCbs_dest, (int) pCbs_end);
			return -1;
			}
//		PRINTI("pwm-stepper debug total1 = %d, total2 = %d, range = %d, next1 = 0x%x, next2 = 0x%x\n", total1, total2, range, next1, next2);
		}
	(--pCbs_dest)->info3 |= DMA_INTEN;  /* enable interupt on last one */
	pCbs_dest->next3 = 0;  /* mark last one as the last */
	return half_steps / 2;  /* return steps */
	}

static NOINLINE void setup_pwm(struct stepper_priv *priv)
	{
	struct S_PWM_REGS *pwm_regs = priv->pwm_regs;

	/* Set up PWM */
	pwm_regs->control.ctl = 0;  /* reset PWM */
//	udelay(10);

	pwm_regs->sta = -1;
//	udelay(10);

	/* enable PWM DMA, raise panic and dreq thresholds to 1 NOTE: setting DREQ(X) will causse
(X+3)/2 glitch cycles (~650 KHz) to come out before our waveform starts */
	pwm_regs->dmac = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(PWM_DMAC_DREQ_VAL);
//	udelay(10);

	pwm_regs->control.ctl = PWM_CTL_CLRF1;  /* clear PWM fifo */
//	udelay(10);

	/* enable PWM channel 1 and use fifo */
	pwm_regs->control.ctl = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;

	}

static NOINLINE void start_dma(struct stepper_priv *priv, struct dma_cb1 *pCbs)
	{
	int index;

	priv->timer_save = *priv->system_timer_regs;  /* save current timer */
	priv->dma_regs->cs = DMA_CHANNEL_RESET;  /* reset DMA */
	priv->pwm_regs->control.ctl = PWM_CTL_CLRF1;  /* clear PWM fifo */

	setup_pwm(priv);
	/* we need to prime the range or we end up with the previous range from the last transfer for the 1st step */
//	PRINTI("pwm-stepper start_dma pCbs = 0x%x 1st Range = 0x%x\n", (int) pCbs, pCbs->range);
	priv->pwm_regs->rng1 = pCbs->range;
/* to get rid of glitch cycles, prime the PWM FIFO with some cycles */

	for (index = 0; index < PWM_DMAC_DREQ_VAL + 2; index++)  /* -1: 2 glitches,
				+ 0: 1 glitch, + 1 gives one glitch but correct timing,
				+ 2 is perfect, + 3: 1st step 2x long, + 5: 4x long */
		priv->pwm_regs->fif1 = 0;

	priv->dma_regs->conblk_ad = (int) pCbs;  /* set start of CB */
	priv->current_dma_buf = pCbs;  /* set new current buf */

	priv->dma_regs->cs = DMA_WAIT_ON_WRITES | DMA_PANIC_PRIORITY(8) |
		DMA_PRIORITY(8) | DMA_ACTIVE;  /* start DMA */
	PRINTI("pwm-stepper start_dma time %d us\n", *priv->system_timer_regs - priv->timer_save);
	}

/* build control block, see page 42 of rpi_DATA_2711_1p0.pdf */
#define BUILD_CB(inf, source, dest, indx, mtr) \
	pCbs->info = inf;	\
	pCbs->src = (int) source;	\
	pCbs->dst = (dest) & PERI_DMA_MASK;	\
	pCbs->length = 4;	\
	pCbs->stride = 0;	\
	pCbs->next = TO_PHYS_KLUDGE(pCbs + 1);	\
	pCbs->motor = mtr;  /* use unused spot to keep track of motor */ \
	pCbs->index = indx;  /* use unused spot to keep track of index */ \
	pCbs++

/* build one motor STEP (6 CB's), NOTE: the ordering of the steps below is really important */
static inline struct dma_cb1 *build1step(struct stepper_priv *priv, struct dma_cb1 *pCbs, int step, u32 half_period, int motor, BUILDTYPE flag)
	{  /* half the period for high and low half-cycles */
	struct pwm_dma_data *dma_send_buf = priv->dma_send_buf;
	int *p_gpio_set_mask = &dma_send_buf->gpio_set_mask[motor];
	int *p_gpio_clr_mask = &dma_send_buf->gpio_clr_mask[motor];
	u32 *p_range = &dma_send_buf->range[step * 2];
	int cb_dx = 0;  /* DMA block index */

//	PRINTI("pwm-stepper debug build1step dx=%d period=%d\n", index, half_period * 2);
	if (flag == USE_DMA_BUF)  /* don't write to range if not a DMA buf build, it will get written during combine */
		*p_range = half_period;
	/* NOTE: CB_RANGE_OFFSET gets set based on the ordering below, set to the relative positioning of the Range write */
	pCbs->range = half_period;  /* save our delay in a secret spot (->src1 in cb3) */
  /* set range on PWM for half_period */
	BUILD_CB(NORMAL_DMA, p_range++, PWM_BASE + PWM_RNG1 * 4, cb_dx++, motor);
  /* delay, half_period range amount via PWM */
	BUILD_CB(NORMAL_DMA | TIMED_DMA(5), p_gpio_set_mask, PWM_BASE + PWM_FIFO1 * 4, cb_dx++, motor);
	/* Set GPIO high */
	BUILD_CB(NORMAL_DMA, p_gpio_set_mask, GPIO_SET, cb_dx++, motor);
	/* Same as above 3 except Set GPIO low */
	if (flag == USE_DMA_BUF)
		*p_range = half_period;
	pCbs->range = half_period;
	BUILD_CB(NORMAL_DMA, p_range, PWM_BASE + PWM_RNG1 * 4, cb_dx++, motor);
	BUILD_CB(NORMAL_DMA | TIMED_DMA(5), p_gpio_clr_mask, PWM_BASE + PWM_FIFO1 * 4, cb_dx++, motor);
	BUILD_CB(NORMAL_DMA, p_gpio_clr_mask, GPIO_CLR, cb_dx++, motor);

	return (pCbs);
	}

/* build DMA control blocks, pass in destination pCB, returns number of steps built or 0 */
static NOINLINE int build_dma_thread(struct stepper_priv *priv, int motor, struct dma_cb1 *pCbs, int flag)
	{
	struct STEPPER_SETUP *p_cmd = &priv->step_cmd[motor];
	struct pwm_dma_data *dma_send_buf = priv->dma_send_buf;
	int *p_gpio_set_mask = &dma_send_buf->gpio_set_mask[motor];
	int *p_gpio_clr_mask = &dma_send_buf->gpio_clr_mask[motor];
	int distance = abs(p_cmd->distance);
	int cntr, half_period, set_mask, clr_mask;
	int step = 0;

//	printk(KERN_ERR "pwm-stepper build_dma_thread dist = %d, motor = %d step = %d pCbs = 0x%x, flag = %d\n",
//		p_cmd->distance, motor, step, (int) pCbs, flag);
  /* first, setup GPIO set/clears */
	set_mask = clr_mask = 1 << p_cmd->gpios[GPIO_STEP];  /* set our step GPIO */
	for (cntr = GPIO_MICROSTEP0; cntr <= GPIO_MICROSTEP2; cntr++) {
		if (p_cmd->microstep_control & (1 << cntr))
			set_mask |= 1 << p_cmd->gpios[cntr];
		else
			clr_mask |= 1 << p_cmd->gpios[cntr];
		}
	if (p_cmd->distance >= 0)
		set_mask |= 1 << p_cmd->gpios[GPIO_DIRECTION];
	else
		clr_mask |= 1 << p_cmd->gpios[GPIO_DIRECTION];
	*p_gpio_set_mask = set_mask;
	*p_gpio_clr_mask = clr_mask;

	half_period = PWM_FREQ / p_cmd->speed / 2;
	for (cntr = min(distance, MAX_STEPS / priv->motors); cntr > 0; cntr--) {
		pCbs = build1step(priv, pCbs, step++, half_period, motor, flag);
		}
	(--pCbs)->next = 0;  /* mark last one as the last */
	pCbs->info |= DMA_INTEN;  /* enable interupt on last one */

	return step;
	}

/* DMA callback from DMA interrupt */
static irqreturn_t bcm2835_dma_callback(int irq, void *data)
	{
	struct stepper_priv *priv = data;

	if (priv->pwm_regs->sta & 0x1fc)
		printk(KERN_ERR "pwm-stepper Error PWM STA = 0x%x\n", priv->pwm_regs->sta);

	priv->dma_regs->cs |= DMA_INTERRUPT_STATUS;  /* Clear the INT flag */
	PRINTI("pwm-stepper debug dma int complete %dms\n", (*priv->system_timer_regs - priv->timer_save) / 1000);
	complete(&priv->dma_cmpl);

	return IRQ_HANDLED;
	}

static ssize_t step_cmd_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buffer, loff_t pos, size_t count)
	{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stepper_priv *priv = dev_get_drvdata(dev);

	priv->step_cmd_read.status = priv->dma_regs->cs;  /* get status */
	memcpy(buffer, &priv->step_cmd_read, sizeof(priv->step_cmd_read));
//	PRINTI("pwm-stepper debug status = 0x%x, size = %d\n", priv->dma_regs->cs, sizeof(priv->step_cmd_read));
	return sizeof(priv->step_cmd_read);
	}

/* take a command from user space, if we are not currently doing a DMA to a motor, just start a new DMA.  If we are already doing a DMA, find out where our current trasfer is (via conblk_ad) and calcuate how far we should move ahead, knowing how long it takes us to copy/build/combine, then start the combine at that point. */
static ssize_t step_cmd_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buffer, loff_t pos, size_t count)
	{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stepper_priv *priv = dev_get_drvdata(dev);
	struct dma_cb1 *next_dma_buf = priv->current_dma_buf;
	struct STEPPER_SETUP *p_cmd = (struct STEPPER_SETUP *) buffer;
	int flag, timer_save, ret, build_steps = 0, combine_steps = 0, last_motor, cntr;
	struct dma_cb1 *build_cbsA = priv->build_cbs2;
	struct dma_cb1 *build_cbsB = priv->build_cbs2;
	struct dma_cb1 *build_cbsC;
#if MAX_MOTORS > 2
 = priv->build_cbs3
#endif
		 ;

	ret = wait_event_interruptible_timeout(priv->wait_q, priv->state == STEPPER_STATE_IDLE, msecs_to_jiffies(50));
	if (!ret)
		return -ETIMEDOUT;
	if (ret < 0)
		return ret; /* got a signal */
	priv->state = STEPPER_STATE_BUSY;
		
	timer_save = *priv->system_timer_regs;  /* save current timer */
	if (priv->gpio2motor[p_cmd->gpios[GPIO_STEP]] == NO_MOTOR)
		{  /* this is a new motor, request gpio's */
		priv->gpio2motor[p_cmd->gpios[GPIO_STEP]] = priv->motors++;  /* set this motor */
		/* Set the GPIO pins */
//		PRINTI("pwm-stepper debug microstep_control = %d, step = %d, motor = %d\n", p_cmd->microstep_control, p_cmd->gpios[GPIO_STEP], priv->gpio2motor[p_cmd->gpios[GPIO_STEP]]);
		ret = request_set_gpio(priv, p_cmd->gpios[GPIO_MICROSTEP0], p_cmd->microstep_control >> 0);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_MICROSTEP1], p_cmd->microstep_control >> 1);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_MICROSTEP2], p_cmd->microstep_control >> 2);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_DIRECTION], p_cmd->distance >= 0 ? 1 : 0);
		ret |= request_set_gpio(priv, p_cmd->gpios[GPIO_STEP], 0);
		if (ret) {
			printk(KERN_ERR "pwm-stepper Requesting GPIO's that are already used\n");
			count = -EBUSY;
			goto bail;
			}
		}
	if (abs(p_cmd->distance) > MAX_STEPS / priv->motors) {
		printk(KERN_ERR "pwm-stepper Exceeding size of %d max steps\n", MAX_STEPS / priv->motors);
		count = -EINVAL;
		goto bail;
		}
	memcpy(&priv->step_cmd[priv->gpio2motor[p_cmd->gpios[GPIO_STEP]]], buffer, count);  /* save this command */

	if (next_dma_buf == priv->dma_send_buf->run_cbs1)  /* ping pong to other dma buffer */
		next_dma_buf = priv->dma_send_buf->run_cbs2;
	else
		next_dma_buf = priv->dma_send_buf->run_cbs1;
	for (last_motor = 0, cntr = 0; cntr < priv->motors; cntr++)  /* find last motor */
		if (priv->step_cmd[cntr].speed != 0)
			last_motor = cntr;  /* count only motors with speed */

	for (cntr = 0; cntr < priv->motors; cntr++) {  /* go thru all motors and build/combine */
		if (priv->step_cmd[cntr].speed == 0)  /* skip if no speed */
			continue;
		flag = USE_BUILD_BUF;
		if (cntr == last_motor && cntr == 0) {
			build_cbsA = next_dma_buf;
			flag = USE_DMA_BUF;
			}
		build_steps = build_dma_thread(priv, cntr, build_cbsA, flag);
		if (build_steps <= 0) {  /* did build fail? */
			report_debug("4");
			printk(KERN_ERR "pwm-stepper build error\n");
			count = -EINVAL;  /* error */
			goto bail;
			}
		build_cbsA = priv->build_cbs1;  /* from now on, build in cbs1 */
		if (cntr == 0)
			continue;  /* don't combine on the first one */
		if (cntr == last_motor) {
			build_cbsC = next_dma_buf;
			flag = USE_DMA_BUF;
			}
		combine_steps = combine_dma_threads(priv, (struct dma_cb3 *) build_cbsC,
			(struct dma_cb3 *) build_cbsA, (struct dma_cb3 *) build_cbsB, flag);
		if (combine_steps <= 0) {  /* did combine fail? */
			report_debug("5");
			printk(KERN_ERR "pwm-stepper combine error\n");
			count = -EINVAL;  /* error */
			goto bail;
			}
		/* swap B and C build bufs */
#if MAX_MOTORS > 2
		if (build_cbsB == priv->build_cbs2)
			build_cbsC = priv->build_cbs2;
		else
			build_cbsB = priv->build_cbs3;
#endif
		}
	PRINTI("pwm-stepper write time %d us\n", *priv->system_timer_regs - timer_save);

	start_dma(priv, next_dma_buf);
	report_debug("G");  /* everything AOK */
bail:
	priv->state = STEPPER_STATE_IDLE;
	wake_up_interruptible(&priv->wait_q);
	return count;
	}

/* shows up in /sys/devices/platform/stepper_plat/cmd */
static const struct bin_attribute step_cmd_attr = {
	.attr = {.name = "cmd", .mode = 0666},
	.size = sizeof(struct STEPPER_SETUP),	/* Limit image size */
	.write = step_cmd_write,
	.read = step_cmd_read,
};

/*
 * set PWM frequency function
 *  Parameters:
 *   freq   - frequency in Hz
 */
static void pwm_frequency(struct stepper_priv *priv, u32 freq)
	{
	u32 divi, divf, pwen1;

	pwen1 = priv->pwm_regs->control.ctl_bits.pwen1;  /* save state */
	priv->pwm_regs->control.ctl_bits.pwen1 = 0;  /* Disable PWM */

	divi = RPI4_CRYSTAL_FREQ / freq;
	divf = RPI4_CRYSTAL_FREQ % freq;

	*(priv->pwm_clk_regs + PWMCLK_CTL) = CLK_PASS | CLK_CTL_KILL;  /* stop the clock */
	*(priv->pwm_clk_regs + PWMCLK_DIV) = CLK_PASS | (divi << 12) | divf;  /* Set the integer divisor */	*(priv->pwm_clk_regs + PWMCLK_CTL) = CLK_PASS | CLK_CTL_SRC(1);  /* Set source to oscillator */
	*(priv->pwm_clk_regs + PWMCLK_CTL) = CLK_PASS | CLK_CTL_ENAB | CLK_CTL_SRC(1);  /* enable clock */
	udelay(10);
	priv->pwm_regs->control.ctl_bits.pwen1 = pwen1;  /* restore PWM */
	}

static int bcm2835_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stepper_priv *priv;
	struct resource *res;
	struct irq_desc *desc;
	int virq, ret = 0;
	void __iomem *pwm_regs;  /* base addr of PWM */

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ERR "pwm-stepper: priv alloc failed, decrease MAX_STEPS\n");
		return -ENOMEM;
		}
	init_waitqueue_head(&priv->wait_q);
	init_completion(&priv->dma_cmpl);
	memset(priv->gpio2motor, NO_MOTOR, sizeof(priv->gpio2motor));  /* set to No Motor */
	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);  /* get PWM reg base ptr */
	pwm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(pwm_regs))
		goto out1;

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "clock not found: %d\n", ret);
		goto out1;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto out2;

	ret = sysfs_create_bin_file(&pdev->dev.kobj, &step_cmd_attr);
	if (ret) {
		printk(KERN_ERR "pwm-stepper: Failed to create sysfs files\n");
		goto out3;
	}

	/* Map the from PHYSICAL address space to VIRTUAL address space */
	priv->pwm_clk_regs = ioremap(PWM_CLK_BASE, PAGE_SIZE);
	priv->system_timer_regs = ioremap(SYSTEM_TIMER_CLO, PAGE_SIZE);
	priv->dma_regs = ioremap(DMA_BASE, BCM2711_DMA_CHANNELS * 256);
	priv->pwm_regs = (struct S_PWM_REGS *)pwm_regs;
	priv->gpio_regs = (struct S_GPIO_REGS *)ioremap(GPIO_BASE, sizeof(struct S_GPIO_REGS));
	if (!priv->gpio_regs || !priv->pwm_clk_regs || !priv->dma_regs)
		{
		printk(KERN_ERR "pwm-stepper: failed to ioremap\n");
		ret = -ENOMEM;
		goto out4;
		}
	priv->dma_chan_tx = dma_request_chan(dev, "rx-tx");
	if (!priv->dma_chan_tx) {
		printk(KERN_ERR "pwm-stepper: failed to dma_request_chan\n");
		goto out4;
		}

#define DMA_CHANNEL 0  /* The channel we get from the last call, be nice to figure this out! */
#define DMA_START_VIRQ_NUM (80 + 32)  /* 80 Gotten from "GIC_SPI 80..." in bcm2711.dtsi */
	for (virq = 0; virq < 64; virq++) {  /* find our interrupt */
		desc = irq_to_desc(virq);
		if (desc->irq_data.hwirq == DMA_START_VIRQ_NUM + DMA_CHANNEL)
			break;
		}
	if (desc->irq_data.hwirq != DMA_START_VIRQ_NUM + DMA_CHANNEL) {
		printk(KERN_ERR "pwm-stepper can't find DMA interrupt %d\n", DMA_START_VIRQ_NUM + DMA_CHANNEL);
		goto out5;
		}
	priv->irq_number = virq;
	priv->dma_regs = &priv->dma_regs[DMA_CHANNEL];  /* move to our reg's */
	if (request_irq(priv->irq_number, bcm2835_dma_callback, 0, "Stepper DMA IRQ", priv)) {
		printk(KERN_ERR "pwm-stepper request_irq %d failed\n", priv->irq_number);
		goto out5;
		}
	/* preallocate dma buffers */
	priv->dma_size = sizeof(struct pwm_dma_data);
	priv->dma_send_buf = dma_alloc_coherent(dev, priv->dma_size,
		&priv->dma_handle, GFP_KERNEL | GFP_DMA);
	printk(KERN_INFO "pwm-stepper: Inserting stepper driver module dma_end_buf = 0x%x, dma_send_buf = 0x%x, priv alloc = %d, dma buf alloc = %d\n", (int) &priv->dma_send_buf->gpio_clr_mask[MAX_MOTORS], (int) priv->dma_send_buf, sizeof(*priv), priv->dma_size);
	if (!priv->dma_send_buf) {
		printk(KERN_ERR "pwm-stepper: dma buf alloc failed, decrease MAX_STEPS\n");
		ret = -ENOMEM;
		goto out6;
		}
	pwm_frequency(priv, PWM_FREQ);
	return 0;

out6:
	free_irq(priv->irq_number, priv);  /* free our int and put his back */
out5:
	dma_release_channel(priv->dma_chan_tx);
out4:
	sysfs_remove_bin_file(&pdev->dev.kobj, &step_cmd_attr);
out3:
	clk_disable_unprepare(priv->clk);
out2:
	devm_clk_put(dev, priv->clk);
out1:
	kfree(priv);

	return ret;
}

static int bcm2835_pwm_remove(struct platform_device *pdev)
{
	struct stepper_priv *priv = platform_get_drvdata(pdev);
	int ret = 0;

	dmaengine_terminate_all(priv->dma_chan_tx);
	dma_free_coherent(&pdev->dev, priv->dma_size, priv->dma_send_buf, priv->dma_handle); 
	free_irq(priv->irq_number, priv);  /* free our int and put his back */
	dma_release_channel(priv->dma_chan_tx);
	sysfs_remove_bin_file(&pdev->dev.kobj, &step_cmd_attr);
	clk_disable_unprepare(priv->clk);
	devm_clk_put(&priv->pdev->dev, priv->clk);
	kfree(priv);
	PRINTI("pwm-stepper debug bcm2835_pwm_remove 3\n");
 	platform_device_unregister(pdev); // hangs, never returns from <mutex_lock>
	PRINTI("pwm-stepper debug bcm2835_pwm_remove 4\n");
	return ret;
}

static const struct of_device_id bcm2835_pwm_of_match[] = {
	{ .compatible = "brcm,bcm2835-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm2835_pwm_of_match);

static struct platform_driver bcm2835_pwm_driver = {
	.driver = {
		.name = "bcm2835-pwm",
		.of_match_table = bcm2835_pwm_of_match,
	},
	.probe = bcm2835_pwm_probe,
	.remove = bcm2835_pwm_remove,
};
module_platform_driver(bcm2835_pwm_driver);
