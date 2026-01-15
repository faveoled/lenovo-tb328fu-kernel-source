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

struct Backlight_Current {
	struct i2c_client *client;
	struct platform_device *mp_dev;
	u32 chipid;
};

static int Bias_slave_addr = 0x36;

static struct Backlight_Current Backlight_Current;
/*
static int Backlight_Current_read(u8 reg, u8 *data, u16 length)
{
	int err;
	struct i2c_client *i2c = Backlight_Current.client;
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
		pr_info("Backlight_Current i2c read error:%d\n", err);
		pr_info("msg:%d-%d:%d, %d-%d:%d\n",
			msgs[0].addr, msgs[0].len, msgs[0].buf[0],
			msgs[1].addr, msgs[1].len, msgs[1].buf[1]);
		return err;
	}

	return length;
}
*/
static int Backlight_Current_write(u8 reg, u8 *data, u16 length)
{
	int err;
	u8 buf[length + 1];
	struct i2c_client *i2c = Backlight_Current.client;
	struct i2c_msg msg;

	buf[0] = reg;
	memcpy(&buf[1], data, length * sizeof(u8));
	msg.flags = 0;
	msg.buf = buf;
	msg.addr = i2c->addr;
	msg.len = length + 1;

	err = i2c_transfer(i2c->adapter, &msg, 1);
	if (err < 0) {
		pr_info("Backlight_Current i2c write error:%d\n", err);
		pr_info("msg:%d-%d:%d\n", msg.addr, msg.len, msg.buf[0]);
		return err;
	}

	return length;
}

void Backlight_Current_writei2c_byte(u8 reg, u8 value)
{
	Backlight_Current_write(reg, &value, 1);
}
/*
static u8 Backlight_Current_readi2c_byte(u8 reg)
{
	u8 value = 0;

	Backlight_Current_read(reg, &value, 1);
	return value;
}
*/
static int Backlight_Current_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("I2C check functionality fail!");
		return -ENODEV;
	}

	pr_info("Bais_slave_addr:0x%X,client->addr:%X\n",
		Bias_slave_addr, client->addr);
	Backlight_Current.client = client;

	pr_info("I2C device probe OK\n");

	return 0;
}

static int Backlight_Current_remove(struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	Backlight_Current.client = NULL;

	return 0;
}

static const struct i2c_device_id Backlight_Current_ids[] = {
	{ "Backlight_drv", 0 },
	{ }
};

static const struct of_device_id Backlight_Current_matchs[] = {
	{ .compatible = "sprd,I2C_LCD_BACKLIGHT", },
	{ }
};

static struct i2c_driver Backlight_Current_driver = {
	.probe		= Backlight_Current_probe,
	.remove		= Backlight_Current_remove,
	.id_table	= Backlight_Current_ids,
	.driver	= {
		.name	= "Backlight_drv",
		.owner	= THIS_MODULE,
		.bus = &i2c_bus_type,
		.of_match_table = Backlight_Current_matchs,
	},
};

static int __init Backlight_Current_init(void)
{
	return i2c_add_driver(&Backlight_Current_driver);
}

static void Backlight_Current_exit(void)
{
	pr_info("Backlight_Current i2c exit\n");
	i2c_del_driver(&Backlight_Current_driver);
}

module_init(Backlight_Current_init);
module_exit(Backlight_Current_exit);

MODULE_AUTHOR("ashley.chen@spreadtrum.com");
MODULE_DESCRIPTION("Backlight_Current_I2C Drivers");
MODULE_LICENSE("GPL");
