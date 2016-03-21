/*
 * lirc_gpio.c
 *
 * lirc_gpio - Device driver that records pulse- and pause-lengths
 *              (space-lengths) (just like the lirc_serial driver does)
 *              between GPIO interrupt events.  Tested on a Cubieboard with Allwinner A10
 *        However, everything relies on the gpiolib.c module, so there is a good
 *        chance it will also run on other platforms.
 *              Lots of code has been taken from the lirc_rpi module, who in turn took a
 *        lot of code from the lirc_serial module,
 *              so I would like say thanks to the authors.
 *
 * Copyright (C) 2013 Matthias H��lling <mhoel....@gmail.nospam.com>,
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>
#include <plat/sys_config.h>
#include <../drivers/gpio/gpio-sunxi.h>

#define LIRC_DRIVER_NAME "lirc_gpio"
/* this may have to be adapted for different platforms */
#define LIRC_GPIO_ID_STRING "A1X_GPIO"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 256

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define RX_OFFSET_GPIOCHIP gpio_in_pin - gpiochip->base
#define TX_OFFSET_GPIOCHIP gpio_out_pin - gpiochip->base

#define dprintk(fmt, args...)                                        \
do {                                                        \
if (debug)                                        \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": "        \
fmt, ## args);                        \
} while (0)

/* module parameters */


/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -1;
static struct timeval lasttv = { 0, 0 };
static spinlock_t lock;

/* set the default pwm num only 1 or 2 or A20 */
static int pwm_num = 1;
/* enable debugging messages */
static int debug;

/* 0 = do not invert output, 1 = invert output */
static int invert = 0;

/* is the device open, so interrupt must be changed if pins are changed */
static int device_open = 0;


struct irq_chip *irqchip=NULL;
struct irq_data *irqdata=NULL;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_send_pwm_exit(void);

static struct platform_device *lirc_send_pwm_dev;
static struct lirc_buffer rbuf;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;


/* stuff for TX pin */

static void safe_udelay(unsigned long usecs)
{
    /*TODO remplacer ceci par quelque chose de plus précis et de moin gourmand en cpu */
        while (usecs > MAX_UDELAY_US) {
                udelay(MAX_UDELAY_US);
                usecs -= MAX_UDELAY_US;
        }
        udelay(usecs);
}

static int init_timing_params(unsigned int new_duty_cycle,
                              unsigned int new_freq)
{

        /*
         * period, pulse/space width are kept with 8 binary places -
         * IE multiplied by 256.
         */
         int ret;
        if softcarrier = 1 {
            if (256 * 1000000L / new_freq * new_duty_cycle / 100 <=
                LIRC_TRANSMITTER_LATENCY)
                    return -EINVAL;
            if (256 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
                LIRC_TRANSMITTER_LATENCY)
                    return -EINVAL;
            duty_cycle = new_duty_cycle;
            freq = new_freq;
            period = 256 * 1000000L / freq;
            pulse_width = period * duty_cycle / 100;
            space_width = period - pulse_width;
            /*printk(KERN_INFO "in init_timing_params, freq=%d pulse=%ld, "
               "space=%ld\n", freq, pulse_width, space_width); */
               ret = 0; //only one return
        }
        else
        {
            /* compute pwm parameter */
            /*TODO mettre de code de calcul PWM ici */
            ret = 0;
        }
        return ret;
}


static long send_pulse(unsigned long length)
{
        if (length <= 0)
                return 0;

                /*TODO remplacer l'appel gpiochip par pwm */
                //gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, !invert);
                safe_udelay(length);
                return 0;
        }
}


static void send_space(long length)
{        /*TODO remplacer l'appel gpiochip par pwm */
        gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, invert);
        if (length <= 0)
                return;
        safe_udelay(length);
}

/* end of TX stuff */



/* RX stuff: Handle interrupt and write vals to lirc buffer */









/* setup pins, rx, tx, interrupts, active low/high.... */





static int setup_tx(int new_out_pin)
{
	int ret;
	/* TODO à utiliser pour faire une machine d'état
   http://www.ibm.com/developerworks/library/l-timers-list/ */
    return ret;
}





/* called when the character device is opened
   timing params initialized and interrupts activated */
static int set_use_inc(void *data)
{
        int result;
        unsigned long flags;

        init_timing_params(duty_cycle, freq);
        /* initialize pulse/space widths */

    //initialize timestamp, would not be needed if no RX
    do_gettimeofday(&lasttv);
    device_open++;



        return 0;
}

/* called when character device is closed */
static void set_use_dec(void *data)
{
        unsigned long flags;
    device_open--;

    if(!irqchip)
        return;

        /* GPIO Pin Falling/Rising Edge Detect Disable */
    spin_lock_irqsave(&lock, flags);
    irqchip->irq_set_type(irqdata, 0);
    irqchip->irq_mask(irqdata);
    spin_unlock_irqrestore(&lock, flags);

    free_irq(gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP), (void *) 0);
    device_open--;
    dprintk("freed IRQ %d\n", gpiochip->to_irq(gpiochip, RX_OFFSET_GPIOCHIP));

}

/* lirc to tx */
static ssize_t lirc_write(struct file *file, const char *buf,
                          size_t n, loff_t *ppos)
{
        int i, count;
        unsigned long flags;

        long delta = 0;
        int *wbuf;
    if (!gpio_out_pin) {
        return -ENODEV;
    }
        count = n / sizeof(int);
        if (n % sizeof(int) || count % 2 == 0)
                return -EINVAL;
        wbuf = memdup_user(buf, n);
        if (IS_ERR(wbuf))
                return PTR_ERR(wbuf);
        spin_lock_irqsave(&lock, flags);
    dprintk("lirc_write called, offset %d",TX_OFFSET_GPIOCHIP);
        for (i = 0; i < count; i++) {
                if (i%2)
                        send_space(wbuf[i] - delta);
                else
                        delta = send_pulse(wbuf[i]);
        }
        gpiochip->set(gpiochip, TX_OFFSET_GPIOCHIP, invert);
        spin_unlock_irqrestore(&lock, flags);
    if (count>11) {
        dprintk("lirc_write sent %d pulses: no10: %d, no11: %d\n",count,wbuf[10],wbuf[11]);
    }
        kfree(wbuf);
        return n;
}

/* interpret lirc commands */
static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
        int result;
        __u32 value;

        switch (cmd) {
        case LIRC_GET_SEND_MODE:
            return -ENOIOCTLCMD;
            break;

        case LIRC_SET_SEND_MODE:
            result = get_user(value, (__u32 *) arg);
            if (result)
                return result;
            /* only LIRC_MODE_PULSE supported */
            if (value != LIRC_MODE_PULSE)
                return -ENOSYS;
            dprintk("Sending stuff on lirc");
            break;

        case LIRC_GET_LENGTH:
            return -ENOSYS;
            break;

        case LIRC_SET_SEND_DUTY_CYCLE:
            result = get_user(value, (__u32 *) arg);
            if (result)
                return result;
            if (value <= 0 || value > 100)
                return -EINVAL;
            dprintk("SET_SEND_DUTY_CYCLE to %d \n", value);
            return init_timing_params(value, freq);
            break;

        case LIRC_SET_SEND_CARRIER:
            result = get_user(value, (__u32 *) arg);
            if (result==0) // pour moi tous resultats diférent de 0 fait sortir vace la modif seul le 0 fait sortir
                return result;
            if (value > 500000 || value < 20000) //TODO mettre à jour avec les capacité hardware
                return -EINVAL;
            dprintk("SET_SEND_CARRIER to %d \n",value);
            return init_timing_params(duty_cycle, value);
            break;

        default:
            return lirc_dev_fop_ioctl(filep, cmd, arg);
        }
        return 0;
}

static const struct file_operations lirc_fops = {
        .owner                = THIS_MODULE,
        .write                = lirc_write,
        .unlocked_ioctl        = lirc_ioctl,
        .read                = lirc_dev_fop_read, // this and the rest is default
        .poll                = lirc_dev_fop_poll,
        .open                = lirc_dev_fop_open,
        .release        = lirc_dev_fop_close,
        .llseek                = no_llseek,
};

static struct lirc_driver driver = {
        .name                = LIRC_DRIVER_NAME,
        .minor                = -1,           // assing automatically
        .code_length        = 1,
        .sample_rate        = 0,
        .data                = NULL,
        .add_to_buf        = NULL,
        .rbuf                = &rbuf,
        .set_use_inc        = set_use_inc,
        .set_use_dec        = set_use_dec,
        .fops                = &lirc_fops,
        .dev                = NULL,
        .owner                = THIS_MODULE,
};

/* end of lirc device/driver stuff */

/* now comes THIS driver, above is lirc */
static struct platform_driver lirc_gpio_driver = {
        .driver = {
                .name   = LIRC_DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};



/* stuff for sysfs*/

static DEFINE_MUTEX(sysfs_lock);


/* TODO a mettre à jour pour pwm */
static ssize_t lirc_pwm_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",gpio_out_pin);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_pwm_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int new_pin;
    ssize_t status;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&new_pin);
    status = setup_tx(new_pin) ? : size;
    mutex_unlock(&sysfs_lock);
    return status;
}
/*fin mise à jour */


/* à mettre à jour */
static ssize_t lirc_invert_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",invert);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_invert_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int try_value;
    ssize_t status=size;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&try_value);
    if ((try_value==0) || (try_value==1)) {
        invert=try_value;
    }
    else
        status = -EINVAL;
    mutex_unlock(&sysfs_lock);
    return status;
}

/*fin */



/* I don't think we need another device, so just put it in the class directory
 * All we need is a way to access some global parameters of this module */

static struct class_attribute lirc_gpio_attrs[] = {
    __ATTR(pwm_num, 0644, lirc_pwm_show, lirc_pwm_store),
    __ATTR(lirc_invert, 0644, lirc_invert_show, lirc_invert_store),
    __ATTR_NULL,
};
static struct class lirc_gpio_class = { //TODOI renomage
    .name = "lirc_gpio",
    .owner = THIS_MODULE,
    .class_attrs = lirc_gpio_attrs,
};

/* end of sysfs stuff */

/* initialize / free THIS driver and device and a lirc buffer*/

static int __init lirc_gpio_init(void)
{
        int result;

        /* Init read buffer. */
        result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
        if (result < 0)
                return -ENOMEM;

        result = platform_driver_register(&lirc_gpio_driver);
        if (result) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": lirc register returned %d\n", result);
                goto exit_buffer_free;
        }

        lirc_gpio_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
        if (!lirc_gpio_dev) {
                result = -ENOMEM;
                goto exit_driver_unregister;
        }

        result = platform_device_add(lirc_gpio_dev);
        if (result)
                goto exit_device_put;

        return 0;

exit_device_put:
        platform_device_put(lirc_gpio_dev);

exit_driver_unregister:
        platform_driver_unregister(&lirc_gpio_driver);

exit_buffer_free:
        lirc_buffer_free(&rbuf);

        return result;
}

static void lirc_gpio_exit(void)
{
    setup_tx(0); // frees gpio_out_pin if set

        platform_device_unregister(lirc_gpio_dev);
        platform_driver_unregister(&lirc_gpio_driver);
        lirc_buffer_free(&rbuf);
}

/* end of stuff for THIS driver/device registration */
/* ignorance of unset pins in setup routines tolerate call if nothing is set up */

/* master init */

static int __init lirc_send_pwm_init_module(void)
{
    int result,temp_in_pin,temp_out_pin;

        result = lirc_gpio_init(); //TODO à renomer
        if (result)
                return result;
    // 'driver' is the lirc driver
        driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
    LIRC_CAN_SET_SEND_CARRIER |
    LIRC_CAN_SEND_PULSE |
    LIRC_CAN_REC_MODE2; // TODO enlever ce qu'il y a en trop

        driver.dev = &lirc_gpio_dev->dev;  // link THIS platform device to lirc driver TODO renomer
        driver.minor = lirc_register_driver(&driver);

        if (driver.minor < 0) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": device registration failed with %d\n", result);
                result = -EIO;
                goto exit_lirc;
        }

        printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");


    /* some hacking to get pins initialized on first used */
    /* setup_tx/rx will not do anything if pins would not change */

    result = setup_tx(temp_out_pin);
    if (result < 0)
        goto exit_lirc;
    /* dito for rx */



    if (device_open) {  // this is unlikely, but well...
        result = set_use_inc((void*) 0);
        if (result<0) {
            goto exit_lirc;
        }
    }

    result=class_register(&lirc_gpio_class);
    if (result) {
        goto exit_lirc;
    }


        return 0;

exit_lirc:
    /* failed attempt to setup_tx/rx sets pin to 0. */
    /* next call with arg 0 will then not do anything -> only one exit routine */
        lirc_gpio_exit();

        return result;
}

static void __exit lirc_gpio_exit_module(void)
{

        lirc_gpio_exit();
    class_unregister(&lirc_gpio_class);

        lirc_unregister_driver(driver.minor);
        printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_gpio_init_module);
module_exit(lirc_gpio_exit_module);

MODULE_DESCRIPTION("Infra-red  blaster driver for PWM-Lib.");
MODULE_DESCRIPTION("Parameters can be set/changed in /sys/class/lirc_gpio");
MODULE_AUTHOR("Matthias Hoelling <mhoel....@gmail.nospam.com");
MODULE_LICENSE("GPL");

module_param(pwm_num, int, S_IRUGO);
MODULE_PARM_DESC(pwm_num, "which PWM used for output");

module_param(invert, int, S_IRUGO);
MODULE_PARM_DESC(invert, "Invert output (0 = off, 1 = on, default off");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
