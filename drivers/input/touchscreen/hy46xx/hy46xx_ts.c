/* drivers/input/touchscreen/hy46xx_ts.c
 *
 * HYCON hy46xx TouchScreen driver.
 *
 * Copyright (c) 2010  Hycon tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
//#include <linux/earlysuspend.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <linux/vmalloc.h>
#include "../tp_suspend.h"
#include "hy46xx_ts.h"
#define hy46xx_ts_suspend	NULL
#define hy46xx_ts_resume	NULL
static const char *hycon_ts_name = "hycon-ts";
struct i2c_client * i2c_hy46xx_connect_client = NULL;
/**************************************************************/
int hy46xx_i2c_Read(struct i2c_client *client, unsigned char *writebuf, int writelen, unchar *readbuf, int readlen)
{
	int ret;
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	}
	else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}

	return ret;
}
/**************************************************************/
int hy46xx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	return ret;
}

/************************************************************/
void hy46xx_reset_tp(struct i2c_client *client)
{
 struct hy46xx_ts_data  *hy46xx_ts = i2c_get_clientdata(client);
 msleep(10);                         // T2: > 10ms
 gpio_direction_output(hy46xx_ts->rst_pin, 0);   // begin select I2C slave addr
 msleep(10);                         // T2: > 10ms
 gpio_direction_output(hy46xx_ts->rst_pin, 1);
 msleep(70);                         // T2: > 10ms
}
/************************************************************/

/**************************************************************/
static int hy46xx_read_Touchdata(struct hy46xx_ts_data *data)
{
	struct ts_event *event = &data->event;
	unsigned char  buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	unsigned char pointid = HY_MAX_ID;
	for(i = 0; i < POINT_READ_BUF; i++)
	{
		buf[i] = 0xff;
	}
	memset(event, 0, sizeof(struct ts_event));
	buf[0] = 0;
	ret = hy46xx_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	#if 0
	for(i = 0; i < 9; i++)
	{
		pr_info("buf[%d]= 0x%x  ", i,buf[i]);
	}
	#endif
	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[HY_TOUCH_ID_POS + HY_TOUCH_STEP * i]) >> 4;
		if (pointid >= HY_MAX_ID)
			break;
		else{
			event->touch_point++;
			event->au16_x[i] =(unsigned short) (buf[HY_TOUCH_X_H_POS + HY_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[HY_TOUCH_X_L_POS + HY_TOUCH_STEP * i];
			event->au16_y[i] =(unsigned short) (buf[HY_TOUCH_Y_H_POS + HY_TOUCH_STEP * i] & 0x0F) <<8 | (s16) buf[HY_TOUCH_Y_L_POS + HY_TOUCH_STEP * i];
			event->au8_touch_event[i] =buf[HY_TOUCH_EVENT_POS + HY_TOUCH_STEP * i] >> 6;
			event->au8_finger_id[i] =(buf[HY_TOUCH_ID_POS + HY_TOUCH_STEP * i]) >> 4;
			}
		#if 0
		pr_info("id=%d event=%d x=%d y=%d\n", event->au8_finger_id[i],event->au8_touch_event[i], event->au16_x[i], event->au16_y[i]);
		#endif
	}

	event->pressure = 0x05;

	return 0;
}
/**************************************************************/
static void hy46xx_report_value(struct hy46xx_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	for (i = 0; i < event->touch_point; i++)
	{
		if((event->au16_x[i] < RESOLUTION_X) && (event->au16_y[i] < RESOLUTION_Y))
		{
			input_mt_slot(data->input_dev, event->au8_finger_id[i]);
			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,true);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,event->pressure);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);
		//			pr_info("pressure=%d id=%d x=%d y=%d\n", event->pressure, event->au8_finger_id[i], event->au16_x[i], event->au16_y[i]);
			}
			else
			{
				uppoint++;
			  input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
			}

		}
		else
		{
			input_mt_slot(data->input_dev, event->au8_finger_id[i]);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,false);
		}
	//			input_sync(data->input_dev);
	}
	if(event->touch_point == uppoint)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	 }
	else
		input_report_key(data->input_dev, BTN_TOUCH, 1);
input_sync(data->input_dev);

}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: hy46xx i2c_client private data
Output:
    None.
*********************************************************/
void hy46xx_irq_disable(struct hy46xx_ts_data *hy46xx_ts)
{
    unsigned long irqflags;

    spin_lock_irqsave(&hy46xx_ts->irq_lock, irqflags);
    if (!hy46xx_ts->irq_is_disable)
    {
        hy46xx_ts->irq_is_disable = 1;
        disable_irq_nosync(hy46xx_ts->client->irq);
    }
    spin_unlock_irqrestore(&hy46xx_ts->irq_lock, irqflags);
}
/*******************************************************
Function:
    Enable irq function
Input:
    ts: hy46xx i2c_client private data
Output:
    None.
*********************************************************/
void hy46xx_irq_enable(struct hy46xx_ts_data *hy46xx_ts)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&hy46xx_ts->irq_lock, irqflags);
    if (hy46xx_ts->irq_is_disable)
    {
        enable_irq(hy46xx_ts->client->irq);
        hy46xx_ts->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&hy46xx_ts->irq_lock, irqflags);
}
/**************************************************************/
static irqreturn_t hy46xx_ts_irq_handler(int irq, void *dev_id)
{
	struct hy46xx_ts_data *hy46xx_ts = dev_id;
	hy46xx_irq_disable(hy46xx_ts);
//	queue_work(hy46xx_wq, &hy46xx_ts->work);
	hy46xx_read_Touchdata(hy46xx_ts);
	hy46xx_report_value(hy46xx_ts);
	hy46xx_irq_enable(hy46xx_ts);
	return IRQ_HANDLED;
}
/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    hy46xx_ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 hy46xx_request_io_port(struct hy46xx_ts_data *hy46xx_ts)
{
    s32 ret = 0;
		ret = gpio_request(hy46xx_ts->rst_pin, "Touch_Driver_Reset_Pin");
		if (ret < 0)
		{
				printk( "hy46xx_probe: Reset Pin Request Failed\n");
				gpio_free(hy46xx_ts->rst_pin);
				return -ENODEV;
		}
		ret = gpio_request(hy46xx_ts->irq_pin, "Touch_Driver_IRQ_Pin");
		if (ret < 0)
		{
			printk( "hy46xx_probe: IRQ Pin Request Failed\n");
				gpio_free(hy46xx_ts->irq_pin);
				return -ENODEV;
		}
		gpio_direction_input(hy46xx_ts->irq_pin);

	  hy46xx_reset_tp(hy46xx_ts->client);
    return ret;
}

/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int hy46xx_ts_early_suspend(struct tp_device *tp_d)
{
    struct hy46xx_ts_data *hy46xx_ts;
    int reg = 0;

    hy46xx_ts = container_of(tp_d, struct hy46xx_ts_data, tp);
    hy46xx_ts->hy46xx_is_suspend = 1;
        hy46xx_irq_disable(hy46xx_ts);

	reg = regulator_disable(hy46xx_ts->tp_regulator);
	if (reg < 0)
	 printk( "Regulator Disable Failed\n");
	msleep(20);
	return 0;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int hy46xx_ts_early_resume(struct tp_device *tp_d)
{
    struct hy46xx_ts_data *hy46xx_ts;
    int reg = 0;
    hy46xx_ts = container_of(tp_d, struct hy46xx_ts_data, tp);
	   reg = regulator_enable(hy46xx_ts->tp_regulator);
	if (reg < 0)
		printk( "Regulator Enable Failed\n");
	  msleep(10);
    hy46xx_irq_enable(hy46xx_ts);
    hy46xx_ts->hy46xx_is_suspend = 0;

	return 0;
}
/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 hy46xx_request_input_dev(struct i2c_client *client,
                                struct hy46xx_ts_data *hy46xx_ts)
{
    s8 ret = -1;
   s8 phys[32];
    hy46xx_ts->input_dev = devm_input_allocate_device(&client->dev);
    if (hy46xx_ts->input_dev == NULL)
    {
			 dev_err(&client->dev, "Input Device Allocation Failed\n");
        return -ENOMEM;
    }

    hy46xx_ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		//input_mt_init_slots(hy46xx_ts->input_dev, 16, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
    hy46xx_ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    __set_bit(INPUT_PROP_DIRECT, hy46xx_ts->input_dev->propbit);

		//input_set_abs_params(hy46xx_ts->input_dev, ABS_X, 0, hy46xx_ts->abs_x_max, 0, 0);
		//input_set_abs_params(hy46xx_ts->input_dev, ABS_Y, 0, hy46xx_ts->abs_y_max, 0, 0);
		input_mt_init_slots(hy46xx_ts->input_dev, CFG_MAX_TOUCH_POINTS,INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
		input_set_abs_params(hy46xx_ts->input_dev, ABS_MT_POSITION_X, 0, (RESOLUTION_X-1), 0, 0);
    input_set_abs_params(hy46xx_ts->input_dev, ABS_MT_POSITION_Y, 0, (RESOLUTION_Y-1), 0, 0);
    input_set_abs_params(hy46xx_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    //sprintf(phys, "input/ts");
    hy46xx_ts->input_dev->name = hycon_ts_name;
    hy46xx_ts->input_dev->phys = phys;
    hy46xx_ts->input_dev->id.bustype = BUS_I2C;
    hy46xx_ts->input_dev->id.vendor = 0xDEAD;
    hy46xx_ts->input_dev->id.product = 0xBEEF;
    hy46xx_ts->input_dev->id.version = 10427;
    ret = input_register_device(hy46xx_ts->input_dev);
    if (ret)
    {
			 dev_err(&client->dev, "Input Device Registration Failed\n");
        return -ENODEV;
    }
		hy46xx_ts->tp.tp_resume = hy46xx_ts_early_resume;
    hy46xx_ts->tp.tp_suspend = hy46xx_ts_early_suspend;
    tp_register_fb(&hy46xx_ts->tp);
    return 0;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 hy46xx_request_irq(struct hy46xx_ts_data *hy46xx_ts)
{
    s32 ret = -1;

    hy46xx_ts->irq=gpio_to_irq(hy46xx_ts->irq_pin);       //If not defined in client
    if (hy46xx_ts->irq)
    {
        hy46xx_ts->client->irq = hy46xx_ts->irq;
        ret = devm_request_threaded_irq(&(hy46xx_ts->client->dev), hy46xx_ts->irq, NULL,
            hy46xx_ts_irq_handler, hy46xx_ts->irq_flags | IRQF_ONESHOT /*irq_table[ts->int_trigger_type]*/,
            hy46xx_ts->client->name, hy46xx_ts);
        if (ret != 0) {
            			 printk( "IRQ allocation Failed\n");
            goto test_pit;
        }

    }else{
       printk("hy46xx_ts->Error\n");
        ret = 1;
        goto test_pit;
    }

test_pit:
    if (ret)
    {
			 printk( "IRQ Request Failed\n");
        gpio_direction_input(hy46xx_ts->irq_pin);
        gpio_free(hy46xx_ts->irq_pin);
        return -1;
    }
    else
    {
        printk( "IRQ Request Succeeded\n");
        hy46xx_irq_disable(hy46xx_ts);
        return 0;
    }
}


/**********************************************************************************/
static int hy46xx_ts_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	s32 ret = -1;
	int err = 0;
	struct hy46xx_ts_data *hy46xx_ts;
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags rst_flags;

  printk("%s() start\n", __func__);
	i2c_hy46xx_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	hy46xx_ts = kzalloc(sizeof(*hy46xx_ts), GFP_KERNEL);
	if (hy46xx_ts == NULL)
	{
			dev_err(&client->dev, "Hy46xx_ts_Probe: Memory Allocation Failed\n");
			err = -ENOMEM;
					goto exit_alloc_data_failed;
	}
  memset(hy46xx_ts, 0, sizeof(*hy46xx_ts));

	if (!np) {
		dev_err(&client->dev, "no device tree\n");
		return -EINVAL;
	}

	hy46xx_ts->tp_regulator = devm_regulator_get(&client->dev, "tp");
	if (IS_ERR(hy46xx_ts->tp_regulator)) {
		dev_err(&client->dev, "failed to get regulator\n");

		return(-1);
	}

	ret = regulator_enable(hy46xx_ts->tp_regulator);
	if (ret < 0)
			dev_err(&client->dev, "failed to enable regulator\n");
	msleep(20);

	hy46xx_ts->irq_pin = of_get_named_gpio_flags(np, "touch-gpio", 0, (enum of_gpio_flags *)(&hy46xx_ts->irq_flags));
	hy46xx_ts->rst_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);
  hy46xx_ts->client = client;

	//INIT_WORK(&hy46xx_ts->work, goodix_ts_work_func);
	//hy46xx_ts->client = client;
	spin_lock_init(&hy46xx_ts->irq_lock);
	i2c_set_clientdata(client, hy46xx_ts);
  hy46xx_ts->hy46xx_rawdiff_mode = 0;
	ret = hy46xx_request_io_port(hy46xx_ts);
	if (ret < 0)
	{
		dev_err(&client->dev, "Failed IO Port requests\n");
    goto exit_request_io_failed;
	}

	ret = hy46xx_request_input_dev(client, hy46xx_ts);
	if (ret < 0)
	{
				dev_err(&client->dev, "failed to allocate input device\n");
	}
ret = hy46xx_request_irq(hy46xx_ts);


 hy46xx_irq_enable(hy46xx_ts);
	return 0;

exit_request_io_failed:
	free_irq(client->irq, hy46xx_ts);
	i2c_set_clientdata(client, NULL);
	kfree(hy46xx_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

/************************************************************/
static int hy46xx_ts_remove(struct i2c_client *client)
{
	struct hy46xx_ts_data *hy46xx_ts;
	hy46xx_ts = i2c_get_clientdata(client);
	input_unregister_device(hy46xx_ts->input_dev);

	#ifdef HYS_APK_DEBUG
		hy46xx_remove_sysfs(client);
		hy_rw_iic_drv_exit();
		hy46xx_release_apk_debug_channel();
	#endif
  free_irq(client->irq, hy46xx_ts);
	kfree(hy46xx_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}
static const struct i2c_device_id hy46xx_ts_id[] = {
	{HY46XX_NAME, 0},
	{}
};
static const struct of_device_id hy46xx_dt_match[] = {
	{.compatible = "hycon,hy46xx_ts"},
	{},
};


static struct i2c_driver hy46xx_ts_driver = {
	.probe = hy46xx_ts_probe,
	.remove = hy46xx_ts_remove,
	.id_table = hy46xx_ts_id,
	//.suspend = hy46xx_ts_suspend,
	//.resume = hy46xx_ts_resume,
	.driver = {
		   .name = HY46XX_NAME,
//		   .owner = THIS_MODULE,
			 	 .of_match_table = of_match_ptr(hy46xx_dt_match),
//		   .of_match_table = hy46xx_dt_match,
		   },
};
/************************************************************/
static int hy46xx_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&hy46xx_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding hy46xx driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			hy46xx_ts_driver.driver.name);
	}
	return ret;
}
/************************************************************/
static void hy46xx_ts_exit(void)
{
	i2c_del_driver(&hy46xx_ts_driver);
}

module_init(hy46xx_ts_init);
module_exit(hy46xx_ts_exit);

MODULE_AUTHOR("<THREEE FIVE DISPLAYS>");
MODULE_DESCRIPTION("HYCON hy46xx TouchScreen driver");
MODULE_LICENSE("GPL");
