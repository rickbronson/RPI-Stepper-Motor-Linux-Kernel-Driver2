/*********************************************************************
 * test-stepper.c - PWM RPI4 test program

To make:
make

To run:
sudo ~/src/test-stepper -s 500 -d 1 | less
sudo ~/src/test-stepper -s 500 -d 1 | grep otal
sudo ~/src/test-stepper -d 1 -n 1004 -s 40000 -m 7 -v -p 13
sudo ~/src/test-stepper -d 1 -n 500 -s 500 -m 7 -i 16 -p 12
 - run both motors:
cd ~/src; while true; do sudo ~/src/test-stepper -l 2 -y 200; sleep 3; done
 - run 1st motor, 2nd motor, then both:
cd ~/src; while true; do sudo ~/src/test-stepper -d 400 -n 500 -s 500 -m 7 -i 16 -p 12; sleep 1; sudo ~/src/test-stepper -l 1 -y 200; sleep 3; sudo ~/src/test-stepper -l 2 -y 200; sleep 3; done

 *********************************************************************/

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
int pthread_tryjoin_np(pthread_t thread, void **retval);
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <time.h>
typedef unsigned char		u8;
typedef unsigned short		u16;
typedef unsigned int		u32;
typedef unsigned long long	u64;
typedef signed char		s8;
typedef short			s16;
typedef int			s32;
typedef long long		s64;
#include "rpi4-stepper.h"

#define ARRAY_SIZE(s) (sizeof(s) / sizeof(*s))

struct dma_cb3 {  /* to make it easier to deal with 3 control blocks that entail one time delay */
	u32 info1;
	u32 *src1;  /* pointer to GPIO value */
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
	u32 *src2;  /* pointer to range value */
	u32 dst2;  /* PWM perif range reg */
	u32 length2;
	u32 stride2;
	u32 next2;
	u32 pad2[2];
	u32 info3;
	u32 src3;  /* any valid pointer */
	u32 dst3;  /* PWM perif FIFO reg */
	u32 length3;
	u32 stride3;
	u32 next3;
	u32 pad3[2];
};

#define PERI_BASE   0xfe000000
#define SYSTEM_TIMER_CLO (PERI_BASE + 0x00003004)  /* based on 1 MHz */

#define STEP_CMD_FILE "/sys/devices/platform/soc/fe20c000.pwm/cmd"

/* A string listing valid short options letters.  */
const char* program_name;  /* The name of this program.  */
const char* const short_options = "d:s:m:p:i:l:a:y:tv";
  /* An array describing valid long options.  */
const struct option long_options[] = {
    { "distance",      1, NULL, 'd' },
    { "speed",     1, NULL, 's' },
    { "microstep",     1, NULL, 'm' },
    { "step_gpio",     1, NULL, 'p' },
    { "dir_gpio",     1, NULL, 'i' },
    { "loop",     1, NULL, 'l' },
    { "parse",     1, NULL, 'a' },
    { "delay",     1, NULL, 'y' },
    { "status",     0, NULL, 't' },
    { "verbose",   0, NULL, 'v' },
    { NULL,        0, NULL, 0   }   /* Required at end of array.  */
  };

void print_usage (FILE* stream, int exit_code)
{
  fprintf (stream, "Usage:  %s options\n", program_name);
  fprintf (stream,
           "  -d  --distance    Distance in >=0 sets direction pin to 0, <0 sets direction pin to 1\n"
           "  -s  --speed        Speed in steps/second\n"
           "  -m  --microstep    Microstep [7]\n"
           "  -p  --step_gpio    Step GPIO [13]\n"
           "  -i  --dir_gpio     Dir GPIO [6]\n"
           "  -l  --loop         Loop count\n"
           "  -a  --parse        Parse address of DMA Control Blocks\n"
           "  -y  --delay        Delay between loops in milliseconds\n"
           "  -t  --status       Get DMA status register\n"
           "  -v  --verbose      Print verbose messages.\n");
  exit (exit_code);
}

struct stepper_priv {
	struct STEPPER_SETUP step_cmd;
	int verbose;
	int loop;
	int last_range[32];
	pthread_mutex_t mutex;  /* Protects critical region */
	pthread_mutexattr_t attr1;
	int write_calls[MAX_MOTORS];
	pthread_t write_thread_id[MAX_MOTORS];
  pthread_attr_t attr[MAX_MOTORS];
	int msdelay;
	int get_status;
	int fd;
	} priv_data = {0};

#define DEFAULT_AGGRESSIVENESS 10  /* lower 8 bits treated as a fractions */
struct STEPPER_SETUP setup[] =
	{
	{
	.distance = 20,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 777,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_13,
	.gpios[GPIO_DIRECTION] = GPIO_06,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = 20,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 500,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_12,
	.gpios[GPIO_DIRECTION] = GPIO_16,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = 1,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 666,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_13,
	.gpios[GPIO_DIRECTION] = GPIO_06,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = -2,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 555,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_12,
	.gpios[GPIO_DIRECTION] = GPIO_16,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = 1,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 600,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_13,
	.gpios[GPIO_DIRECTION] = GPIO_06,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = 1,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 650,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_12,
	.gpios[GPIO_DIRECTION] = GPIO_16,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = -7,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 700,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_13,
	.gpios[GPIO_DIRECTION] = GPIO_06,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	{
	.distance = 0,  /* in steps NOTE: signed, 
										pos = DIR pin high, neg = DIR pin low NOTE: if = 0 then stop */
	.speed = 650,  /* max speed in steps/second NOTE: if = 0 then stop */
	.microstep_control = 7,  /* bit 0 is value for gpio_microstep0, bit 1 = microstep1, etc */
	.gpios[GPIO_STEP] = GPIO_12,
	.gpios[GPIO_DIRECTION] = GPIO_16,
	.gpios[GPIO_MICROSTEP0] = GPIO_19,
	.gpios[GPIO_MICROSTEP1] = GPIO_20,
	.gpios[GPIO_MICROSTEP2] = GPIO_21,
	},
	};

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
static int map_read_mem(off_t addr)  /* map memory */
  {
  void *map_base; 
  void volatile *virt_addr; 
  int fd, retval;

  if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
		printf("Error, can't open /dev/mem, need to run as sudo\n");
    return (fd);
		}
  /* Map one page */
  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
  if (map_base == (void *) -1)
    return -1;

  virt_addr = map_base + (addr & MAP_MASK);
  retval = *((unsigned long *) virt_addr);
  if (munmap(map_base, MAP_SIZE) == -1)
    return -1;
	close(fd);
  return (retval);
  }

#define SZ_4K				0x00001000
#define SZ_8K			0x00002000
#define SZ_16K		0x00004000
#define SZ_32K		0x00008000
#define SZ_64K		0x00010000
#define SZ_1M			0x00100000
#define SZ_2M			0x00200000
#define PARSE_MAP_SIZE SZ_1M
#define PARSE_MAP_MASK (PARSE_MAP_SIZE - 1)
#define PA_MASK 0x1fffffff  /* kludge to get phys address */

/* parse passed address that points to DMA Control blocs eg "-a 0xfe003000 */
static int parse_cbs(struct stepper_priv *priv, off_t parse)
	{
  void *map_base, *virt_addr; 
  struct dma_cb3 *pCbs;
	int fd, gpio, range, freq, cntr2, cntr = 1;
	char *dst;
	
  if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    return (fd);
  /* Map one page */
  map_base = mmap(0, PARSE_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, parse & ~PARSE_MAP_MASK);
  if (map_base == (void *) -1)
    return -1;

  pCbs = (struct dma_cb3 *) (map_base + (parse & PARSE_MAP_MASK));
	do {
		gpio = map_read_mem((off_t) pCbs->src3 & PA_MASK);
		gpio = 31 - __builtin_clz(gpio);  /* get bit */
		range = map_read_mem((off_t) pCbs->src1 & PA_MASK);
		dst = "ERR ";
		freq = 0;
		if (pCbs->dst3 == 0x7e20001c)
			dst = "HIGH";
		else
			if (pCbs->dst3 == 0x7e200028) {
				dst = "LOW ";
				if (priv->last_range[gpio]) {
					freq = (1000000 * 27) / priv->last_range[gpio];
					priv->last_range[gpio] = 0;
					}
				}
		if (range / 27)
			printf("CB %3d, info=%02x, next=%08x, gpio(%s)=%2d, range(%x) = %4d us (%d)\n",
				cntr, pCbs->info3 & 0xff, pCbs->next3, dst, gpio, pCbs->src1, range / 27, freq);
		else
			printf("CB %3d, info=%02x, next=%08x, gpio(%s)=%2d, range(%x) = %4d ticks\n",
				cntr, pCbs->info3 & 0xff, pCbs->next3, dst, gpio, pCbs->src1, range, freq);
		for (cntr2 = 0; cntr2 < 32; cntr2++)  /* add range to all motors */
			priv->last_range[cntr2] += range;
		pCbs++;
		cntr++;
		if (cntr > PARSE_MAP_SIZE / sizeof(*pCbs)) {
			printf("Reached parse map size limit, seems to be 1 MB\n");
			break;
			}
		}
	while ((pCbs - 1)->next3);
	
  if (munmap(map_base, MAP_SIZE) == -1)
    return -1;
  return 0;
	}

static void *write_thread(int motor)  /* one for each motor */
  {
  pthread_attr_t  attr = {0};
  int pol = SCHED_FIFO;
  int loop_cntr, rc;
	int system_timer_regs;
	struct timespec ts = { 0, 5000000 };
	struct stepper_priv *priv = &priv_data;
	struct STEPPER_SETUP step_cmd;
	struct STEPPER_SETUP *p_cmd = &step_cmd;

  rc = pthread_attr_init( &attr );
  if ( rc == -1 ) 
    {
    perror( "pthread_attr_init()" ) ;
    return( (void *)errno ) ;
    }
  rc = pthread_attr_getschedpolicy(&attr, &pol);

  if ( rc == -1 )
    {
    perror("pthread_attr_getschedpolicy()");
    return((void *)errno) ;
    }

	if (priv->verbose) {
		printf ("started write_thread%d\n", motor);
		}
	memcpy(p_cmd, &setup[motor], sizeof(struct STEPPER_SETUP));
	for (loop_cntr = 0; loop_cntr < priv->loop; loop_cntr++) {
		system_timer_regs = map_read_mem(SYSTEM_TIMER_CLO);

		pthread_mutex_lock(&priv->mutex);
		lseek(priv->fd, 0, SEEK_SET);
//		if (priv->verbose)
//			printf("gpio = %d, distance = %d, speed = %d msdelay %d\n", p_cmd->gpios[GPIO_STEP], p_cmd->distance, p_cmd->speed, priv->msdelay);
		if (write(priv->fd, p_cmd, sizeof(*p_cmd)) != sizeof(*p_cmd)) {
			perror(STEP_CMD_FILE);
			exit(1);
			}
		pthread_mutex_unlock(&priv->mutex);
		priv->write_calls[motor]++;
		system_timer_regs = map_read_mem(SYSTEM_TIMER_CLO) - system_timer_regs;
//		if (priv->verbose)
//			printf("write time = %d us\n", system_timer_regs);
		/* delay in milliseconds */
		ts.tv_nsec = (priv->msdelay % 1000) * 1000 * 1000;
		ts.tv_sec = priv->msdelay / 1000;
		nanosleep(&ts, NULL);
		}
  return((void *) 0) ;
  }
/*
 * Main program:
 */
int main(int argc,char **argv) {
  int next_option;
	int err, parse = 0, cntr, steps, write_calls_total, write_calls_save[MAX_MOTORS] = {0};
	struct stepper_priv *priv = &priv_data;
	struct timespec ts = { 1, 0 };
	
  do
    {
    next_option = getopt_long (argc, argv, short_options,
                               long_options, NULL);
    switch (next_option)
      {
      case 'd':   /* -d or --distance */
				if (abs(setup[cntr].distance) > MAX_STEPS / MAX_MOTORS) {
					printf("Exceeding distance > %d max steps\n", MAX_STEPS / MAX_MOTORS);
					exit;
					}
				for (cntr = 0; cntr < ARRAY_SIZE(setup); cntr++)
					setup[cntr].distance = strtol (optarg, NULL, 0);
        break;
      case 's':   /* -s or --speed */
				if (strtoul (optarg, NULL, 0) > 250000) {
					printf("Warning: Speed of over 250000 is outside the spec of most motor drivers\n");
					exit;
					}
				for (cntr = 0; cntr < ARRAY_SIZE(setup); cntr++)
					setup[cntr].speed = strtoul (optarg, NULL, 0);
				break;
      case 'm':   /* -m or --microstep */
				for (cntr = 0; cntr < ARRAY_SIZE(setup); cntr++)
					setup[cntr].microstep_control = strtoul (optarg, NULL, 0);
        break;
      case 'p':   /* -p or --step_gpio */
				for (cntr = 0; cntr < ARRAY_SIZE(setup); cntr++)
					setup[cntr].gpios[GPIO_STEP] = strtoul (optarg, NULL, 0);
        break;
      case 'i':   /* -i or --dir_gpio */
				for (cntr = 0; cntr < ARRAY_SIZE(setup); cntr++)
					setup[cntr].gpios[GPIO_DIRECTION] = strtoul (optarg, NULL, 0);
        break;
      case 'l':   /* -l or --loop */
        priv->loop = strtoul (optarg, NULL, 0);
        break;
      case 'a':   /* -a or --parse */
        parse = strtoul (optarg, NULL, 0) & PA_MASK;
        break;
      case 'y':   /* -y or --delay */
        priv->msdelay = strtol (optarg, NULL, 0);
        break;
      case 't':   /* -t or --status */
        priv->get_status = 1;
        break;
      case 'v':   /* -v or --verbose */
        priv->verbose = 1;
        break;
      case '?':   /* The user specified an invalid option.  */
        /* Print usage information to standard error, and exit with exit
           code one (indicating abonormal termination).  */
        print_usage (stderr, 1);
        break;
      case -1:    /* Done with options.  */
        break;
      default:    /* Something else: unexpected.  */
				break;
			}
    }
  while (next_option != -1);

  if (priv->verbose)
    printf ("RPI4 PWM motor driver tester\n");

	pthread_mutex_init(&priv->mutex, &priv->attr1);
  for (cntr = 0; cntr < MAX_MOTORS; cntr++) {
		err = pthread_attr_init( &priv->attr[cntr] );
		if ( err == -1 ) 
			{
			perror( "pthread_attr_init()" ) ;
			return( errno ) ;
			}
		err = pthread_attr_setschedpolicy( &priv->attr[cntr], SCHED_FIFO );
  
		if ( err == -1 ) 
			{
			perror( "pthread_attr_setschedpolicy()" ) ;
			return( errno ) ;
			}
		if ((err = pthread_create (&priv->write_thread_id[cntr], &priv->attr[cntr], (void *) write_thread, (void *) cntr)) != 0)
			{
			printf("Create error!\n");
			return(err);
			}
		}
	if (parse) {
		return(parse_cbs(priv, (off_t) parse));
		}
	priv->fd = open(STEP_CMD_FILE, O_RDWR | O_SYNC);  /* might need root access */
	if ( priv->fd < 0 ) {
		perror(STEP_CMD_FILE);
		exit(1);
	}
  while (1)
    {
    if (priv->verbose)
      {
			for (cntr = 0, write_calls_total = 0; cntr < MAX_MOTORS; cntr++) {
				write_calls_total += priv->write_calls[cntr] - write_calls_save[cntr];
				write_calls_save[cntr] = priv->write_calls[cntr];
				}
			if (priv->get_status) {
				if (read(priv->fd, &priv->step_cmd, sizeof(priv->step_cmd)) != sizeof(priv->step_cmd)) {
					perror(STEP_CMD_FILE);
					close(priv->fd);
					exit(1);
					}
				printf("write calls = %d DMA status reg = 0x%08x\n", write_calls_total, priv->step_cmd.status);
				}
			else
				printf ("write calls = %d\n", write_calls_total);
      }
		nanosleep(&ts, NULL);
		for (err = 0, cntr = 0; cntr < MAX_MOTORS; cntr++) {
			err |= pthread_tryjoin_np(priv->write_thread_id[cntr], NULL); /* succeeds (returns 0) if and only if the thread has exited. */
			}
		if (!err)
			break;
		}
	close(priv->fd);
	return 0;
	}
