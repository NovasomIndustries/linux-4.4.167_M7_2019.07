/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define MAX_TOUCHES		10
#define DEFAULT_POLL_PERIOD	20
#define MAX_X			((1 << 12) - 1)
#define MAX_Y			((1 << 12) - 1)

/* Touchscreen commands */
#define TOUCH_INFO	0x00

struct finger {
	u8 xy;
	u8 x;
	u8 y;
	u8 c_sum;
} __packed;

struct touchdata {
	u8 packet_id;
	struct finger finger[MAX_TOUCHES];
	u8 key;
	u8 checksum;
} __packed;


struct ili2117a {
	struct i2c_client *client;
	struct input_dev *input;
	bool (*get_pendown_state)(void);
	unsigned int poll_period;
	struct delayed_work dwork;
	int max_touch;
};

static int ili2117a_read_reg(struct i2c_client *client, u8 reg, void *buf,
			    size_t len)
{
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(&client->dev, "i2c transfer failed\n");
		return -EIO;
	}

	return 0;
}

static bool ili2117a_report_events(struct input_dev *input,
				  const struct touchdata *touchdata,
				  int max_touch)
{
	int i;
	bool touch;
	bool pen_down = 0;
	u16 x = 0xfff;
	u16 y = 0xfff;
	const struct finger *finger;

	for (i = 0; i < max_touch; i++) {
		touch = 0;
		input_mt_slot(input, i);
		finger = &touchdata->finger[i];

		x = finger->x | (((finger->xy & 0xf0) >> 4) << 8);
		y = finger->y | ((finger->xy & 0x0f) << 8);

		/* This driver returns 0xfff if there is no touch */
		if (x != 0xfff && y != 0xfff) {
			touch = 1;
			pen_down = 1;
		}

		input_mt_report_slot_state(input, MT_TOOL_FINGER, touch);

		if (touch) {
			input_report_abs(input, ABS_MT_POSITION_X, x);
			input_report_abs(input, ABS_MT_POSITION_Y, y);
		}
	}

	input_mt_report_pointer_emulation(input, false);
	input_sync(input);
	return pen_down;
}

static void ili2117a_work(struct work_struct *work)
{
	struct ili2117a *priv = container_of(work, struct ili2117a,
					    dwork.work);
	struct i2c_client *client = priv->client;
	struct touchdata touchdata;
	int error;
	bool status;

	error = ili2117a_read_reg(client, TOUCH_INFO,
				  (uint8_t *) &touchdata, sizeof(touchdata));

	if (error) {
		dev_err(&client->dev,
			"Unable to get touchdata, err = %d\n", error);
		return;
	}

	status = ili2117a_report_events(priv->input, &touchdata,
					priv->max_touch);

	if (status) {
		schedule_delayed_work(&priv->dwork,
				      msecs_to_jiffies(priv->poll_period));
	}
}

static irqreturn_t ili2117a_irq(int irq, void *irq_data)
{
	struct ili2117a *priv = irq_data;

	schedule_delayed_work(&priv->dwork, 0);

	return IRQ_HANDLED;
}

static struct attribute *ili2117a_attributes[] = {
	NULL,
};

static const struct attribute_group ili2117a_attr_group = {
	.attrs = ili2117a_attributes,
};

static int ili2117a_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ili2117a *priv;
	struct input_dev *input;
	int error;
	int poll_period;
	int max_touch;

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	input = input_allocate_device();
	if (!priv || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	priv->client = client;
	priv->input = input;

	error = device_property_read_u32(dev, "ili2117a,poll-period",
					 &poll_period);
	priv->poll_period = error ? DEFAULT_POLL_PERIOD : poll_period;

	error = device_property_read_u32(dev, "ili2117a,max-touch",
					 &max_touch);
	if (max_touch > MAX_TOUCHES)
		max_touch = MAX_TOUCHES;
	priv->max_touch = error ? MAX_TOUCHES : max_touch;

	INIT_DELAYED_WORK(&priv->dwork, ili2117a_work);

	/* Setup input device */
	input->name = "ili2117a Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, MAX_Y, 0, 0);

	/* Multi touch */
	input_mt_init_slots(input, priv->max_touch, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);

	i2c_set_clientdata(client, priv);

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					  ili2117a_irq,
					  IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					  client->name, priv);

	if (error) {
		dev_err(dev, "Failed to request irq, err: %d\n", error);
		goto err_free_mem;
	}

	error = sysfs_create_group(&dev->kobj, &ili2117a_attr_group);
	if (error) {
		dev_err(dev, "Unable to create sysfs attributes, err: %d\n",
			error);
		goto err_free_irq;
	}

	error = input_register_device(priv->input);
	if (error) {
		dev_err(dev, "Cannot register input device, err: %d\n", error);
		goto err_remove_sysfs;
	}

	device_init_wakeup(dev, 1);
	return 0;

err_remove_sysfs:
	sysfs_remove_group(&dev->kobj, &ili2117a_attr_group);
err_free_irq:
	free_irq(client->irq, priv);
err_free_mem:
	input_free_device(input);
	kfree(priv);
	return error;
}

static int ili2117a_i2c_remove(struct i2c_client *client)
{
	struct ili2117a *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &ili2117a_attr_group);
	free_irq(priv->client->irq, priv);
	cancel_delayed_work_sync(&priv->dwork);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

static int __maybe_unused ili2117a_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused ili2117a_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ili2117a_i2c_pm,
			 ili2117a_i2c_suspend, ili2117a_i2c_resume);

static const struct of_device_id ili_match_table[] = {
	{ .compatible = "ilitek,ili2117a" },
	{ },
};

static const struct i2c_device_id ili2117a_i2c_id[] = {
	{ "ili2117a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ili2117a_i2c_id);

static struct i2c_driver ili2117a_ts_driver = {
	.driver = {
		.name = "ili2117a_i2c",
		.pm = &ili2117a_i2c_pm,
		.of_match_table = of_match_ptr(ili_match_table),
	},
	.id_table = ili2117a_i2c_id,
	.probe = ili2117a_i2c_probe,
	.remove = ili2117a_i2c_remove,
};

module_i2c_driver(ili2117a_ts_driver);

MODULE_AUTHOR("Adam Ford <adam.ford@logicpd.com>");
MODULE_DESCRIPTION("ili2117a I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
