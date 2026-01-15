/*
 * Copyright (C) 2019 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/stat.h>

struct Bias_Power {
	struct i2c_client *client;
	struct platform_device *mp_dev;
	u32 chipid;
};

static int Bias_slave_addr = 0x3E;

static struct Bias_Power Bias_Power;
/*
static int Bias_Power_read(u8 reg, u8 *data, u16 length)
{
	int err;
	struct i2c_client *i2c = Bias_Power.client;
	struct i2c_msg msgs[2];

	msgs[0].flags = 0;
	msgs[0].buf = &reg;
	msgs[0].addr = i2c->addr;
	msgs[0].len = 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].addr = i2c->addr;
	msgs[1].len = length;

	err = i2c_transfer(i2c->adapter, msgs, 2);
	if (err < 0) {
		pr_info("Bias_Power i2c read error:%d\n", err);
		pr_info("msg:%d-%d:%d, %d-%d:%d\n",
			msgs[0].addr, msgs[0].len, msgs[0].buf[0],
			msgs[1].addr, msgs[1].len, msgs[1].buf[1]);
		return err;
	}

	return length;
}
*/
static int Bias_Power_write(u8 reg, u8 *data, u16 length)
{
	int err;
	u8 buf[length + 1];
	struct i2c_client *i2c = Bias_Power.client;
	struct i2c_msg msg;

	buf[0] = reg;
	memcpy(&buf[1], data, length * sizeof(u8));
	msg.flags = 0;
	msg.buf = buf;
	msg.addr = i2c->addr;
	msg.len = length + 1;

	err = i2c_transfer(i2c->adapter, &msg, 1);
	if (err < 0) {
		pr_info("Bias_Power i2c write error:%d\n", err);
		pr_info("msg:%d-%d:%d\n", msg.addr, msg.len, msg.buf[0]);
		return err;
	}

	return length;
}

void Bias_Power_writei2c_byte(u8 reg, u8 value)
{
	Bias_Power_write(reg, &value, 1);
}
/*
static u8 Bias_Power_readi2c_byte(u8 reg)
{
	u8 value = 0;

	Bias_Power_read(reg, &value, 1);
	return value;
}
*/
static int Bias_Power_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("I2C check functionality fail!");
		return -ENODEV;
	}

	pr_info("Bais_slave_addr:0x%X,client->addr:%X\n",
		Bias_slave_addr, client->addr);
	Bias_Power.client = client;

	pr_info("I2C device probe OK\n");

	return 0;
}

static int Bias_Power_remove(struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	Bias_Power.client = NULL;

	return 0;
}

static const struct i2c_device_id Bias_Power_ids[] = {
	{ "Bias_Power_drv", 0 },
	{ }
};

static const struct of_device_id Bias_Power_matchs[] = {
	{ .compatible = "sprd,I2C_LCD_BIAS", },
	{ }
};

static struct i2c_driver Bias_Power_driver = {
	.probe		= Bias_Power_probe,
	.remove		= Bias_Power_remove,
	.id_table	= Bias_Power_ids,
	.driver	= {
		.name	= "Bias_Power_drv",
		.owner	= THIS_MODULE,
		.bus = &i2c_bus_type,
		.of_match_table = Bias_Power_matchs,
	},
};

static int __init Bias_Power_init(void)
{
	return i2c_add_driver(&Bias_Power_driver);
}

static void Bias_Power_exit(void)
{
	pr_info("Bias_Power i2c exit\n");
	i2c_del_driver(&Bias_Power_driver);
}

module_init(Bias_Power_init);
module_exit(Bias_Power_exit);

MODULE_AUTHOR("ashley.chen@spreadtrum.com");
MODULE_DESCRIPTION("Bias_Power_I2C Drivers");
MODULE_LICENSE("GPL");
