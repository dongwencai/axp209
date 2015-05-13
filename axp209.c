/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include "axp209.h"
#include "axp209_event.h"
#include "mt7620a_reg.h"

#define GPIO_IRQ_NUM    33

struct {
    dev_t dev;
    struct cdev dev_c;
    struct i2c_client *client;
    struct class *cdev_class;
    struct work_struct work;
    unsigned char irq_status[5];
    unsigned int irq_mask;
}axp209_info;

extern struct notifier_block rgbled_notifier;

static RAW_NOTIFIER_HEAD(axp209_chain);

const unsigned char reg_map[] = {21, 20, 6, 12, 13, 12, 6, 5, 47};

struct file_operations axp209_fops;

int axp209_read_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char *value);

int axp209_write_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char value);

int axp209_open (struct inode *, struct file *);

int axp209_release (struct inode *, struct file *);

long axp209_ioctl (struct file *, unsigned int, unsigned long);

static void axp209_irq_status_clear();

static void send_irq_status(struct work_struct*data);

static int call_axp209_notifiers(unsigned long val, void *v)
{
    return raw_notifier_call_chain(&axp209_chain, val, v);
}
EXPORT_SYMBOL(call_axp209_notifiers);

static int register_axp209_notifier(struct notifier_block *nb)
{
    return raw_notifier_chain_register(&axp209_chain, nb);
}
EXPORT_SYMBOL(register_axp209_notifier);

static int unregister_axp209_notifier(struct notifier_block *nb)
{
    return raw_notifier_chain_unregister(&axp209_chain, nb); 
}
EXPORT_SYMBOL(unregister_axp209_notifier);

static int axp209_sendData(struct i2c_client *client, unsigned char reg,unsigned char value)
{
    int ret = 0;
    int times = 0;
    ret = axp209_write_a8_d8(client,reg,value);
    while(ret != 0 && times < 2)
    {
        ret = axp209_write_a8_d8(client,reg,value);
        times ++;
    }
    return ret;
}
static int axp209_readData(struct i2c_client *client, unsigned char reg,unsigned char *value)
{
    int ret = 0;
    int times = 0;

    ret = axp209_read_a8_d8(client,reg,value);
    while(ret != 0 && times < 2)
    {
        ret = axp209_read_a8_d8(client,reg,value);
        times ++;
    }
    return ret;
}
int axp209_read_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char *value)
{
    unsigned char data[2];
    struct i2c_msg msg[2];
    int ret;

    data[0] = addr;
    data[1] = 0xee;
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &data[0];
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &data[1];

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret >= 0) 
    {
        *value = data[1];
        ret = 0;
    } 
    return ret;
}
int axp209_write_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char value)
{
    struct i2c_msg msg;
    unsigned char data[2];
    int ret;
    
    data[0] = addr;
    data[1] = value;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 2;
    msg.buf = data;

    ret = i2c_transfer(client->adapter, &msg, 1);
    return ret;
}

int axp209_open (struct inode *i, struct file *f)
{
    return 0;
}

int axp209_release (struct inode *i, struct file *f)
{
    return 0;
}

long axp209_ioctl (struct file *i, unsigned int cmd, unsigned long arg)
{
    unsigned int context = 0x0000;
    unsigned char value = 0x00;
    switch(cmd)
    {
        case AXP209_GET_REG:
            copy_from_user((void *)&context,(void __user *)arg, sizeof(unsigned short));
            axp209_readData(axp209_info.client, context >> 8, &value);
            context |= value;
            copy_to_user((void __user *)arg,(void *)&context, sizeof(unsigned short));
            break;
        case AXP209_SET_REG:
            copy_from_user((void *)&context,(void __user *)arg, sizeof(unsigned short));
            axp209_sendData(axp209_info.client, context >> 8, context & 0xff);
            break;
        case AXP209_SET_IRQ_MASK:
            copy_from_user((void *)&context,(void __user *)arg, sizeof(unsigned int));
            axp209_info.irq_mask |= context;
            break;
        case AXP209_CLEAR_IRQ_MASK:
            copy_from_user((void *)&context,(void __user *)arg, sizeof(unsigned int));
            axp209_info.irq_mask &= ~context;
            break;
        case AXP209_GET_IRQ_MASK:
            copy_to_user((void __user *)arg,(void *)&axp209_info.irq_mask, sizeof(unsigned int));
            break;
        case AXP209_GET_ELEC:
            axp209_readData(axp209_info.client, 0xB9, &value);
            context |= value;
            context = value *100 / 0x7f;
            copy_to_user((void __user *)arg,(void *)&context, sizeof(unsigned char));
            break;
        default :
            break;
    }
    return 0;
}

static void axp209_irq_status_clear()
{
    axp209_sendData(axp209_info.client, 0x48, 0xff);
    axp209_sendData(axp209_info.client, 0x49, 0xff);
    axp209_sendData(axp209_info.client, 0x4a, 0xff);
    axp209_sendData(axp209_info.client, 0x4b, 0xff);
    axp209_sendData(axp209_info.client, 0x4c, 0xff);
}

static void axp209_irq_status_read()
{
    int cnt;
    for(cnt = 0; cnt < 5; cnt ++)
    {
        axp209_readData(axp209_info.client, 0x48 + cnt, &axp209_info.irq_status[cnt]);
        printk("reg:%x\t%x\n",0x48 + cnt, axp209_info.irq_status[cnt]);
    }
}

irqreturn_t axp209_irq(int irq, void *data)
{
    axp209_irq_status_read();
    schedule_work(&axp209_info.work);
    return IRQ_RETVAL(IRQ_HANDLED); 
}

static void  usb_power_on()
{
   //gpio 61 62 63 output high 
    *(volatile unsigned int*)GPIO_MODE |= (0x01 << 10); 
    *(volatile unsigned int*)GPIO_71_40_DIR |= (0x07 << 21);
    *(volatile unsigned int*)GPIO_71_40_DATA |= (0x07 << 21);
}

#if 0
static void    gpio33_led_on()
{
    *(volatile unsigned int*)GPIO_MODE |= (0x01 << 9); 
    *(volatile unsigned int*)GPIO_39_24_DIR |= (0x01 << 9);
    *(volatile unsigned int*)GPIO_39_24_DATA |= (0x01 << 9);
}

static void    gpio33_led_off()
{
    *(volatile unsigned *)GPIO_39_24_DATA &= ~(0x01 << 9);
}
#endif

void gpio33_irq_init()
{
    *(volatile unsigned int*)GPIO_MODE |= (0x01 << 9); 
    *(volatile unsigned int*)GPIO_39_24_DIR &= ~(0x01 << 9);
    //*(volatile unsigned int*)GPIO_39_24_INT |= (0x01 << 9); 
    //*(volatile unsigned int*)GPIO_39_24_EDGE |= (0x01 << 9); 
    *(volatile unsigned int*)GPIO_39_24_RMASK &= ~(0x01 << 9); 
    *(volatile unsigned int*)GPIO_39_24_FMASK |= (0x01 << 9); 
    *(volatile unsigned int*)INTENA |= RALINK_INTCTL_PIO; 
}

static unsigned int irq_status_reg_to_event()
{
    unsigned int event = 0x00, cnt;
    for(cnt = 0; cnt < 32 && (axp209_info.irq_mask >> cnt); cnt ++)
    {
       if((axp209_info.irq_mask >> cnt | 0x01) && (axp209_info.irq_status[reg_map[cnt] / 10] & (0x01 << reg_map[cnt] % 10)) )
       {
           event = (0x01 << cnt);
           break;
       }
    }
    return event;
}



static void send_irq_status(struct work_struct*data)
{
    char context[15], cnt;
    static unsigned long prev_event_time = 0;
    unsigned int event = 0x00;
    char *msg[2] = {context, NULL};

    event = irq_status_reg_to_event();
    if(event)
    {
        sprintf(msg[0], "%d", event);
        //PEK 抖动
        if(event & AXP209_PEK_PRESS)
        {
            if(jiffies - prev_event_time < 10)
                goto out;
            prev_event_time = jiffies;
        }
        kobject_uevent_env(&axp209_info.client->dev.kobj, KOBJ_CHANGE, msg);
    }

out:
    axp209_irq_status_clear();
}
static void axp209_irq_enable()
{
    //timer
    axp209_sendData(axp209_info.client, 0x44, 0x80);
}

int precent2level(uint8_t val)
{
    const static int map[2][4] = {{ELEC80_100, ELEC50_80, ELEC20_50, ELEC0_20}, {ELEC80_100_C, ELEC50_80_C, ELEC20_50_C, ELEC0_20_C}};
    int idx0 = 0, idx1 = 0;
    uint8_t value = 0x00;
    axp209_readData(axp209_info.client, 0x00, &value);
    if(value & (1 << 2))
        idx0 = 1;
    switch(val)
    {
        case 80 ... 100:
            idx1 = 0;
            break;
        case 50 ... 79:
            idx1 = 1;
            break;
        case 20 ... 49:
            idx1 = 2;
            break;
        case 0 ... 19:
            idx1 = 3;
            break;
        default:
            break;
    }
    return map[idx0][idx1];
}
static void elec_check(void *dummy)
{
   uint8_t value; 
    int ev = 0x00;
    static int oldev = 0x00;
   while(1)
   {
       schedule_timeout(10000 * HZ);
       axp209_readData(axp209_info.client, 0xB9, &value);
       value = value *100 / 0x7f;
       ev = precent2level(value);
       if (ev != oldev)
       {
           call_axp209_notifiers(ev, NULL);
           oldev = ev;
       }
   }
}

static int axp209_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    int ret;
    if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
        return -ENODEV;
    //gpio33_led_on();
    gpio33_irq_init();
    usb_power_on();
    ret = alloc_chrdev_region(&axp209_info.dev, 0, 1, AXP209_DEV_NAME );
    if(ret)    return ret;
    cdev_init(&axp209_info.dev_c, &axp209_fops);
    ret = cdev_add(&axp209_info.dev_c, axp209_info.dev, 1);
    if(ret)
    {
        unregister_chrdev_region(axp209_info.dev, 1);
        return ret;
    }
    axp209_info.cdev_class = class_create(THIS_MODULE, AXP209_DEV_NAME);
    if(IS_ERR(axp209_info.cdev_class))
    {
        unregister_chrdev_region(axp209_info.dev, 1);
        return -1; 
    }
    device_create(axp209_info.cdev_class, NULL, axp209_info.dev, 0, AXP209_DEV_NAME);
    axp209_sendData(client, 0x40, 0xFE);
    axp209_info.client = client;
    axp209_info.irq_mask = 0x1ff;
    axp209_irq_status_clear();
    axp209_irq_enable();
    register_axp209_notifier(&rgbled_notifier);
    INIT_WORK(&axp209_info.work, send_irq_status);
    request_threaded_irq(gpio_to_irq(GPIO_IRQ_NUM), axp209_irq, NULL, IRQF_TRIGGER_FALLING,"axp209", NULL);
    kthread_run(elec_check, NULL, "axp209_elec_chk");
    return 0;
}

static int axp209_remove(struct i2c_client *client)
{
    device_destroy(axp209_info.cdev_class, axp209_info.dev);
    class_destroy(axp209_info.cdev_class);
    unregister_chrdev_region(axp209_info.dev, 1);
    free_irq(gpio_to_irq(GPIO_IRQ_NUM), NULL);
    unregister_axp209_notifier(&rgbled_notifier);
    //gpio33_led_off();
    return 0;
}

static const struct i2c_device_id axp209_id[] = {
    {"axp209",0},
    { },
};
MODULE_DEVICE_TABLE(i2c,axp209_id);

static const struct of_device_id axp209_of_match[] = {
    {.compatible = "x-powers,axp209"},
    { }
};
MODULE_DEVICE_TABLE(of,axp209_of_match);

static struct i2c_driver axp209_driver = {
    .driver          = {
        .name    = "axp209",
        .owner	 = THIS_MODULE,
        .of_match_table = of_match_ptr(axp209_of_match)
    },
    .probe		 = axp209_probe,
    .remove          = axp209_remove,
    .id_table        = axp209_id
};

struct file_operations axp209_fops = {
    .open = axp209_open,
    .release = axp209_release,
    .unlocked_ioctl = axp209_ioctl,
};

module_i2c_driver(axp209_driver);

MODULE_AUTHOR("mrdong <mrdong@focalcrest.com>");
MODULE_DESCRIPTION("AXP209 I2C bus driver");
MODULE_LICENSE("GPL");
