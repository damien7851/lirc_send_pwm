/*
 * lirc_send_pwm.c
 *
 * lirc_send_pwm - Device driver that records pulse- and pause-lengths
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA*/


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
#include <plat/sys_config.h>
#include <linux/pwm.h>
/* hight resolution timer */
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#define LIRC_DRIVER_NAME "lirc_send_pwm"
/* this may have to be adapted for different platforms */

#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 256

#define MS_TO_NS(x)	(x * 1E6L)
#define US_TO_NS(x)	(x * 1E3L)

#define dprintk(fmt, args...)           \
do {                                    \
if (debug)                              \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": " \
fmt, ## args);                          \
} while (0)

/* module parameters */
/* enable debugging messages */
static int debug = 0;
/* set the default pwm num only 0 or 1 or A20 */
static int pwm_num = 0;
/* 1 = active state is hight, 0 = active state is low */
static int active_state = 1;

static spinlock_t lock;
struct pwm_device *pwm_out;

/* is the device open, so interrupt must be changed if pins are changed */
static int device_open = 0;

/* forward declarations */
static void lirc_send_pwm_exit(void);

static struct platform_device *lirc_send_pwm_dev;
static struct lirc_buffer rbuf;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static int *wbuf; //provient de lirc write puisque'on doit acceder depuis le callsback
static int wbuflength;
//TODO a complété
static const int end = 200;
static struct hrtimer hr_timer;
static int state; //state of sending

/* stuff for TX pin */
enum hrtimer_restart statemachine( struct hrtimer *timer )
{
   


    if (state == end || state == wbuflength)
    {
        pwm_disable(pwm_out);
        kfree(wbuf);
        printk (KERN_INFO LIRC_DRIVER_NAME":sending finished or truncated at 200");
        return HRTIMER_NORESTART;

    } else{
        if (IS_ERR(wbuf))
        {
            printk (KERN_ERR LIRC_DRIVER_NAME":sending called before buffer initialized");
            return HRTIMER_NORESTART;
        }
        if (state%2)  //si impaire
        {
            pwm_disable(pwm_out);
        } else
            pwm_enable(pwm_out);


    
        hrtimer_forward_now(&hr_timer,ns_to_ktime(US_TO_NS(wbuf[state])));
        state ++;
        return HRTIMER_RESTART;
    }
}
//fin TODO

static int init_timing_params(unsigned int new_duty_cycle,
                              unsigned int new_freq)
{
    int ret;
    period = 1000000000L / freq;
    pulse_width = period * duty_cycle / 100;
    ret = pwm_config(pwm_out,period,pulse_width);
    if (ret) {
        printk(KERN_ERR LIRC_DRIVER_NAME ":config pwm fail period or duty mismatch");
    }
    dprintk("pwm is configured with %d \% duty and %d Hz",new_duty_cycle,new_freq);
    return ret;
}

static int setup_tx(unsigned int pwm)
{
    int result;
    ktime_t ktime;
    state = end; // on initaulise la machine d'état sur arret.
        if (pwm == 0 || pwm ==1) {
            if (pwm_out==NULL){
                pwm_out = pwm_request(pwm, "Ir-pwm-out");
                }else{
                pwm_free(pwm_out);
                pwm_out = pwm_request(pwm, "Ir-pwm-out");
                }

            if (IS_ERR(pwm_out)) {
                printk(KERN_ERR LIRC_DRIVER_NAME "pwm request fail returned %d",PTR_ERR(pwm_out));
                goto fail;
            }
            result = init_timing_params(duty_cycle,freq);
            //initialisation timer

            unsigned long delay_in_ms = 200L; // on met un délais initial

            printk("HR Timer module installing\n");

            ktime = ktime_set( 0, MS_TO_NS(delay_in_ms) ); //TODO to delete

            hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
            hr_timer.function = &statemachine; //déclaration du callback timer
            hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL ); //TODO a supprimer pout test uniquement
            return result;
        } else if (pwm == -1 ) {
            goto fail_conf;
        }

fail_conf:
    free_pwm(pwm_out);
fail:
    return result;
}

/* end of TX stuff */


/* called when the character device is opened
   timing params initialized and interrupts activated */
static int set_use_inc(void *data)
{
    int ret;
    ret = init_timing_params(duty_cycle, freq);
    //TODO add initalisation hrtimer
    device_open++; //utile?
    return ret;
}

/* called when character device is closed */
static void set_use_dec(void *data)
{
    pwm_disable(pwm_out);
    device_open--; //utile ?
    //TODO add free hrtimer
}

/* lirc to tx */
static ssize_t lirc_write(struct file *file, const char *buf,
                          size_t n, loff_t *ppos)
{
        state = 0;
        wbuflength = n / sizeof(int);
        if (n % sizeof(int) || wbuflength % 2 == 0)
                return -EINVAL;
        wbuf = memdup_user(buf, n);
        if (IS_ERR(wbuf))
                return PTR_ERR(wbuf);
        dprintk("lirc_write called");
       
        hrtimer_start( &hr_timer, ns_to_ktime(US_TO_NS(wbuf[0])), HRTIMER_MODE_REL );
        pwm_enable(pwm_out);
        state ++;
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
            if (result)
                return result;
            if (value > 500000 || value < 20000) /* this value is widely understood in*/
                                                 /*the material ability but is a real IR modulation range*/
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
static struct platform_driver lirc_send_pwm_driver = {
        .driver = {
                .name   = LIRC_DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};



/* stuff for sysfs*/

static DEFINE_MUTEX(sysfs_lock);

static ssize_t lirc_pwm_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",pwm_num);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_pwm_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int new_pwm;
    ssize_t status;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&new_pwm);
    status = setup_tx(new_pwm) ? -EINVAL : size;
    mutex_unlock(&sysfs_lock);
    return status;
}
/*fin mise à jour */



static ssize_t lirc_active_state_show(struct class *class, struct class_attribute *attr, char *buf)
{
    ssize_t status;
    mutex_lock(&sysfs_lock);
    status = sprintf(buf,"%d\n",active_state);
    mutex_unlock(&sysfs_lock);
    return status;
}

static ssize_t lirc_active_state_store(struct class *class, struct class_attribute *attr, const char* buf, size_t size)
{
    int try_value;
    ssize_t status=size;
    mutex_lock(&sysfs_lock);
    sscanf(buf,"%d",&try_value);
    if ((try_value==0) || (try_value==1)) {
    //    pwm_polarity(pwm_out,try_value);
    }
    else
        status = -EINVAL;
    mutex_unlock(&sysfs_lock);
    return status;
}


/* I don't think we need another device, so just put it in the class directory
 * All we need is a way to access some global parameters of this module */

static struct class_attribute lirc_send_pwm_attrs[] = {
    __ATTR(pwm_num, 0644, lirc_pwm_show, lirc_pwm_store),
    __ATTR(lirc_active_state, 0644, lirc_active_state_show, lirc_active_state_store),
    __ATTR_NULL,
};
static struct class lirc_send_pwm_class = { //TODOI renomage
    .name = "lirc_send_pwm",
    .owner = THIS_MODULE,
    .class_attrs = lirc_send_pwm_attrs,
};

/* end of sysfs stuff */

/* initialize / free THIS driver and device and a lircconvert_string_to_microseconds buffer*/

static int __init lirc_send_pwm_init(void)
{
        int result;
        int active_periode;

        /* Init read buffer. */
        result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
        if (result < 0)
                return -ENOMEM;

        result = platform_driver_register(&lirc_send_pwm_driver);
        if (result) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": lirc register returned %d\n", result);
                goto exit_buffer_free;
        }

        lirc_send_pwm_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
        if (!lirc_send_pwm_dev) {
                result = -ENOMEM;
                goto exit_driver_unregister;
        }

        result = platform_device_add(lirc_send_pwm_dev);
        if (result)
                goto exit_device_put;
        //configuration pwm

        result = setup_tx(pwm_num);
        if (result){
            printk(KERN_ERR LIRC_DRIVER_NAME "setup failed returned %d\n",result);
            goto pwm_free_exit;

        }


        return 0;

pwm_free_exit:
        setup_tx(-1);

exit_device_add:
        platform_device_unregister(lirc_send_pwm_dev); /*TODO : verify this line */

exit_device_put:
        platform_device_put(lirc_send_pwm_dev);

exit_driver_unregister:
        platform_driver_unregister(&lirc_send_pwm_driver);

exit_buffer_free:
        lirc_buffer_free(&rbuf);

        return result;
}

static void lirc_send_pwm_exit(void)
{
    //désactivation timer
    int ret;
    ret = hrtimer_cancel( &hr_timer );
    if (ret) printk("The timer was still in use...\n");
    printk("HR Timer module uninstalling\n");
    //fin timer
    pwm_disable(pwm_out);
    pwm_free(pwm_out);
    platform_device_unregister(lirc_send_pwm_dev);
    platform_driver_unregister(&lirc_send_pwm_driver);
    lirc_buffer_free(&rbuf);
}

/* end of stuff for THIS driver/device registration */
/* ignorance of unset pins in setup routines tolerate call if nothing is set up */

/* master init */

static int __init lirc_send_pwm_init_module(void)
{
    int result,temp_in_pin,temp_out_pin;

        result = lirc_send_pwm_init();
        if (result)
                return result;
    // 'driver' is the lirc driver
        driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
    LIRC_CAN_SET_SEND_CARRIER |
    LIRC_CAN_SEND_PULSE ; // TODO enlever ce qu'il y a en trop

        driver.dev = &lirc_send_pwm_dev->dev;  // link THIS platform device to lirc driver TODO renomer
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

     if (device_open) {  // this is unlikely, but well...
        result = set_use_inc((void*) 0);
        if (result<0) {
            goto exit_lirc;
        }
    }

    result=class_register(&lirc_send_pwm_class);
    if (result) {
        goto exit_lirc;
    }


        return 0;

exit_lirc:
    /* failed attempt to setup_tx/rx sets pin to 0. */
    /* next call with arg 0 will then not do anything -> only one exit routine */
        lirc_send_pwm_exit();

        return result;
}

static void __exit lirc_send_pwm_exit_module(void)
{

        lirc_send_pwm_exit();
    class_unregister(&lirc_send_pwm_class);

        lirc_unregister_driver(driver.minor);
        printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_send_pwm_init_module);
module_exit(lirc_send_pwm_exit_module);

MODULE_DESCRIPTION("Infra-red  blaster driver for PWM-Lib.");
MODULE_DESCRIPTION("Parameters can be set/changed in /sys/class/lirc_send_pwm");
MODULE_AUTHOR("Matthias Hoelling <mhoel....@gmail.nospam.com");
MODULE_AUTHOR("Damien Pageot <damien...@gmail.com");
MODULE_LICENSE("GPL");

module_param(pwm_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pwm_num, "which PWM used for output : 0 or 1");

module_param(active_state, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(active_state, "Active state of pwm : 0=active low, 1=active hight");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
