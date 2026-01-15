/*
 *  driver interface to finger print sensor  for 
 *	Copyright (c) 2015  ChipSailing Technology.
 *	All rights reserved.
***********************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/string.h>

//int board_id_version;
unsigned long board_id;
unsigned long board_id_get_boardid(void)
{
	return board_id;
}
EXPORT_SYMBOL(board_id_get_boardid);

int __init setup_boardid(char *str)
{
	board_id = simple_strtoul(str, NULL, 16);
	pr_info("board_id: 0x%x\n", board_id);
	return 1;
}
__setup("androidboot.lc.boardid=", setup_boardid);
