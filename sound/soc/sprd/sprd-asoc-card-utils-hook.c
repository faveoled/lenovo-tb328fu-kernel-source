/*
 * ASoC SPRD sound card support
 *
 * Copyright (C) 2015 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "sprd-asoc-debug.h"
#define pr_fmt(fmt) pr_sprd_fmt("BOARD")""fmt

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "sprd-asoc-card-utils.h"
#include "sprd-asoc-common.h"

struct sprd_asoc_ext_hook_map {
	const char *name;
	sprd_asoc_hook_func hook;
	int en_level;
};
/* ext_pa_disbale begin */
static int disabled_ext_pa_id = -1; // default both enable
                                    //   -1:  enable all;
                                    //   >=0: disable pa[disabled_ext_pa_id];
static int hook_state[BOARD_FUNC_MAX] = {-1}; // record each hook on/off status

enum {
	/* ext_ctrl_type */
	CELL_CTRL_TYPE,
	/* pa type select */
	CELL_HOOK,
	/* select mode */
	CELL_PRIV,
	/* share gpio with  */
	CELL_SHARE_GPIO,
	CELL_NUMBER,
};

struct sprd_asoc_hook_spk_priv {
	int gpio[BOARD_FUNC_MAX];
	int priv_data[BOARD_FUNC_MAX];
	spinlock_t lock;
};

static struct sprd_asoc_hook_spk_priv hook_spk_priv;

// add by Foursemi 20210419 start
static int adp_gpio, adp_status;
static ssize_t fsm_adp_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buff)
{
	return sprintf(buff, "%d\n", adp_status);
}

static ssize_t fsm_adp_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buff, size_t len)
{
	unsigned long adp_mode;
	int ret;

	ret = kstrtoul(buff, 10, &adp_mode);
	if (ret) {
		pr_err("%s kstrtoul failed!(%d)\n", __func__, ret);
		return len;
	}
	adp_status = adp_mode;
	gpio_set_value(adp_gpio, adp_status);
	
	
	pr_info("fsm speaker ext pa adp_status = %d\n", adp_status);

	return len;
}
// add by Foursemi 20210419 end


#define GENERAL_SPK_MODE 10

#define EN_LEVEL 1

#define FS_AW_PA
#ifdef FS_AW_PA
#define FS1512N_START  300
#define FS1512N_PULSE_DELAY_US 20
#define FS1512N_T_WORK  300
#define FS1512N_T_PWD  260
#define SPC_GPIO_HIGH 1
#define SPC_GPIO_LOW 0
#define FS1512N_RETRY          (10)

#define AW87XXX_PULSE_DELAY_US 5
#define AW87XXX_T_PWD  1000
extern unsigned long board_id_get_boardid(void);
int fs15xx_shutdown(unsigned int gpio);
int aw87xxx_shutdown(unsigned int gpio);
static int mode = 1;
static int gpio_last = 0;
static bool pa_is_aw = true;

static ssize_t extpa_info_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buff)
{
	return sprintf(buff, "%s\n", pa_is_aw?"awxxx":"fs15xx");
}

int hook_gpio_pulse_control_FS1512N(unsigned int gpio,int mode_new)
{
	unsigned long flags;
	spinlock_t *lock = &hook_spk_priv.lock;
	int count;
	int ret = 0;
	
	sp_asoc_pr_info("%s gpio:%d-->%d mode: %d-->%d\n",__func__, gpio_last,gpio,mode,mode_new);
	
	if (mode == mode_new && gpio_last == gpio) {
		// the same mode and same gpio, not to switch again
		return ret;
	} else {
		// switch mode online, need shut down pa firstly
		fs15xx_shutdown(gpio);
	}
	
	mode = mode_new;
	gpio_last = gpio; 
	// enable pa into work mode
	// make sure idle mode: gpio output low
	gpio_direction_output(gpio, 0);
	spin_lock_irqsave(lock, flags);
	// 1. send T-sta	
	gpio_set_value( gpio, SPC_GPIO_HIGH);
	udelay(FS1512N_START); 
	gpio_set_value( gpio, SPC_GPIO_LOW);
	udelay(FS1512N_PULSE_DELAY_US); // < 140us
	// 2. send mode
	count = mode - 1;
	while (count > 0) { // count of pulse
		gpio_set_value( gpio, SPC_GPIO_HIGH);
		udelay(FS1512N_PULSE_DELAY_US); // < 140us 10-150
		gpio_set_value( gpio, SPC_GPIO_LOW);
		udelay(FS1512N_PULSE_DELAY_US); // < 140us
		count--;
	}

	// 3. pull up gpio and delay, enable pa
	gpio_set_value( gpio, SPC_GPIO_HIGH);
	// check_intervel_time(fs15xx, 500, 1950);
	spin_unlock_irqrestore(lock, flags);

	udelay(FS1512N_T_WORK); // pull up gpio > 220us
	//fs15xx->spc_mode = mode;

	return ret;
}


int fs15xx_shutdown(unsigned int gpio)
{
    spinlock_t *lock = &hook_spk_priv.lock;
    unsigned long flags;

    spin_lock_irqsave(lock, flags);
    mode = 1;
    gpio_last = 0;
    gpio_set_value( gpio, SPC_GPIO_LOW);
    udelay(FS1512N_T_PWD);
    spin_unlock_irqrestore(lock, flags);
    return 0;
}
int hook_gpio_pulse_control_aw87xxx(unsigned int gpio,int mode_new)
{
	unsigned long flags;
	spinlock_t *lock = &hook_spk_priv.lock;
	int count;
	int ret = 0;
	
	sp_asoc_pr_info("%s gpio:%d-->%d mode: %d-->%d\n",__func__, gpio_last,gpio,mode,mode_new);
	
	if (mode == mode_new && gpio_last == gpio) {
		// the same mode and same gpio, not to switch again
		return ret;
	} else {
		// switch mode online, need shut down pa firstly
		aw87xxx_shutdown(gpio);
	}
	
	mode = mode_new;
	gpio_last = gpio; 
	gpio_direction_output(gpio, 0);
	spin_lock_irqsave(lock, flags);
	//send mode
	count = mode - 1;
	while (count > 0) { // count of pulse
		gpio_set_value( gpio, SPC_GPIO_HIGH);
		udelay(AW87XXX_PULSE_DELAY_US); // < 5us 0.75~10
		gpio_set_value( gpio, SPC_GPIO_LOW);
		udelay(AW87XXX_PULSE_DELAY_US); // < 5us 0.75~10
		count--;
	}

	gpio_set_value( gpio, SPC_GPIO_HIGH);
	spin_unlock_irqrestore(lock, flags);
	return ret;
}


int aw87xxx_shutdown(unsigned int gpio)
{
    spinlock_t *lock = &hook_spk_priv.lock;
    unsigned long flags;

    spin_lock_irqsave(lock, flags);
    mode = 1;
    gpio_last = 0;
    gpio_set_value( gpio, SPC_GPIO_LOW);
    udelay(AW87XXX_T_PWD);//1ms
    spin_unlock_irqrestore(lock, flags);
    return 0;
}

#endif
static int select_mode;

static ssize_t select_mode_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buff)
{
	return sprintf(buff, "%d\n", select_mode);
}

static ssize_t select_mode_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buff, size_t len)
{
	unsigned long level;
	int ret;


	ret = kstrtoul(buff, 10, &level);
	if (ret) {
		pr_err("%s kstrtoul failed!(%d)\n", __func__, ret);
		return len;
	}
	select_mode = level;
	pr_info("speaker ext pa select_mode = %d\n", select_mode);

	return len;
}

static int ext_debug_sysfs_init(void)
{
	int ret;
	static struct kobject *ext_debug_kobj;
	static struct kobj_attribute ext_debug_attr =
		__ATTR(select_mode, 0644,
		select_mode_show,
		select_mode_store);
// add by Foursemi 20210419 start
	static struct kobj_attribute fsm_adp_attr =
		__ATTR(fsm_adp, 0644,
		fsm_adp_show,
		fsm_adp_store);

	static struct kobj_attribute extpa_info_attr =
		__ATTR(extpa_info, 0664,
		extpa_info_show,
		NULL);
// add by Foursemi 20210419 end 

	if (ext_debug_kobj)
		return 0;
	ext_debug_kobj = kobject_create_and_add("extpa", kernel_kobj);
	if (ext_debug_kobj == NULL) {
		ret = -ENOMEM;
		pr_err("register sysfs failed. ret = %d\n", ret);
		return ret;
	}

	ret = sysfs_create_file(ext_debug_kobj, &ext_debug_attr.attr);
	if (ret) {
		pr_err("create sysfs failed. ret = %d\n", ret);
		return ret;
	}

// add by Foursemi 20210419 start
	ret = sysfs_create_file(ext_debug_kobj, &extpa_info_attr.attr);
	if (ret) {
		pr_err("create sysfs failed. ret = %d\n", ret);
		return ret;
	}
	ret = sysfs_create_file(ext_debug_kobj, &fsm_adp_attr.attr);
	if (ret) {
		pr_err("create sysfs failed. ret = %d\n", ret);
		return ret;
	}
// add by Foursemi 20210419 end
	return ret;
}
#ifndef FS_AW_PA
static void hook_gpio_pulse_control(unsigned int gpio, unsigned int mode)
{
	int i = 1;
	spinlock_t *lock = &hook_spk_priv.lock;
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	for (i = 1; i < mode; i++) {
		gpio_set_value(gpio, EN_LEVEL);
		udelay(2);
		gpio_set_value(gpio, !EN_LEVEL);
		udelay(2);
	}

	gpio_set_value(gpio, EN_LEVEL);
	spin_unlock_irqrestore(lock, flags);
}
#endif
static int hook_general_spk(int id, int on)
{
	int gpio, mode;
	
	/* skip disabled_ext_pa_id */
        if (disabled_ext_pa_id == id) {
        pr_info("%s id: %d, disabled by 'ext_pa_disbale'\n",
            __func__, id);
        return HOOK_OK;
        }

	gpio = hook_spk_priv.gpio[id];
	if (gpio < 0) {
		pr_err("%s gpio is invalid!\n", __func__);
		return -EINVAL;
	}
	mode = hook_spk_priv.priv_data[id];
	if (mode > GENERAL_SPK_MODE)
		mode = 0;
	pr_info("%s id: %d, gpio: %d, mode: %d, on: %d\n",
		 __func__, id, gpio, mode, on);

	/* update pa status */
        hook_state[id] = on;

	/* Off */
	if (!on) {
#ifdef FS_AW_PA
	if(!pa_is_aw){
		if (adp_status == 1){
		gpio_set_value(adp_gpio,0);
		adp_status = 0;
		}
		fs15xx_shutdown(gpio);
	}else
	aw87xxx_shutdown(gpio);
#else
		gpio_set_value(gpio, !EN_LEVEL);
#endif
		return HOOK_OK;
	}

	/* On */
	if (select_mode) {
		mode = select_mode;
		pr_info("%s mode: %d, select_mode: %d\n",
			__func__, mode, select_mode);
	}
#ifdef FS_AW_PA
	if(!pa_is_aw){
		pr_info("It is FS1512N");
		hook_gpio_pulse_control_FS1512N(gpio,mode);
	}
	else{ 
		pr_info("It is aw87318");
		hook_gpio_pulse_control_aw87xxx(gpio,mode);
	}
#else
	hook_gpio_pulse_control(gpio, mode);
#endif
	/* When the first time open speaker path and play a very short sound,
	 * the sound can't be heard. So add a delay here to make sure the AMP
	 * is ready.
	 */
	msleep(22);

	return HOOK_OK;
}
#if 0
// for 2in1 ext pa
static int hook_general_ctl(int id, int on)
{
	static int rcv_on = -1, spk_on = -1;
	if (0 != id && 2 != id) {
		pr_info("%s id: %d not support!\n", __func__, id);
		return HOOK_OK;
	}

	if ((0 == id && on == spk_on)
		|| (2 == id && on == rcv_on)) {
		pr_info("%s id: %d, already set: %d\n", __func__, id, on);
		return HOOK_OK;
	}

	// if both spk and rcv set to on, use spk mode
	if (on) {
		if (0 == id) {
			if (rcv_on) {
				hook_general_spk(0, 0);
				hook_general_spk(2, 0);
			}
			hook_general_spk(0, 1);
		        hook_general_spk(2, 1); // if there are two ext-speakers
			spk_on = 1;
		} else {
			if (!spk_on) {
				hook_general_spk(0, 1);
				hook_general_spk(2, 1);
			}
			rcv_on = 1;
		}
	} else {
		if (0 == id) {
			hook_general_spk(0, 0);
		        hook_general_spk(2, 0); // if there are two ext-speakers
			if (rcv_on) {
				hook_general_spk(0, 1);
				hook_general_spk(2, 1);
			}
			spk_on = 0;
		} else {
			if (!spk_on) {
				hook_general_spk(0, 0);
				hook_general_spk(2, 0);
			}
			rcv_on = 0;
		}
	}
	
	pr_info("%s spk %d; rcv %d", __func__, spk_on, rcv_on);
	return 0;
}
#endif
int sprd_ext_pa_disbale_put(
    struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mc =
        (struct soc_mixer_control *)kcontrol->private_value;
    int max = mc->max, id = -1;
    unsigned int mask = (1 << fls(max)) - 1;
    sp_asoc_pr_info("%s put 0x%lx\n",
        __func__, ucontrol->value.integer.value[0]);
    id = (ucontrol->value.integer.value[0] & mask)-1;

    /*
     * if pa[disabled_ext_pa_id] is already on,
     * we need to close it now
     */
    if (id >= 0) {
        if (hook_state[id]) {
            hook_general_spk(id, 0);
            sp_asoc_pr_info("%s id %d off\n",
                __func__, id);
        }
    }

    // update disabled_ext_pa_id
    disabled_ext_pa_id = id;
    sp_asoc_pr_info("%s id %d disabled\n",
        __func__, disabled_ext_pa_id);

    return 0;
}

int sprd_ext_pa_disbale_get(
    struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_value *ucontrol)
{
    sp_asoc_pr_info("%s get %d\n",
        __func__, disabled_ext_pa_id);
    ucontrol->value.integer.value[0] = disabled_ext_pa_id+1;
    return 0;
}
/* ext_pa_disbale end */

static struct sprd_asoc_ext_hook_map ext_hook_arr[] = {
	//{"general_speaker", hook_general_spk, EN_LEVEL},
	{"general_speaker", hook_general_spk, EN_LEVEL},
};

static int sprd_asoc_card_parse_hook(struct device *dev,
					 struct sprd_asoc_ext_hook *ext_hook)
{
	struct device_node *np = dev->of_node;
	const char *prop_pa_info = "sprd,spk-ext-pa-info";
	const char *prop_pa_gpio = "sprd,spk-ext-pa-gpio";
	int spk_cnt, elem_cnt, i;
	int ret = 0;
	unsigned long gpio_flag;
	unsigned int ext_ctrl_type, share_gpio, hook_sel, priv_data;
	u32 *buf;
	// add by Foursemi 20210419 start
	const char *prop_pa_adp_gpio = "sprd,spk-ext-pa-adp-pin";
// add by Foursemi 20210419 end
	elem_cnt = of_property_count_u32_elems(np, prop_pa_info);
	if (elem_cnt <= 0) {
		dev_info(dev,
			"Count '%s' failed!(%d)\n", prop_pa_info, elem_cnt);
		return -EINVAL;
	}

	if (elem_cnt % CELL_NUMBER) {
		dev_err(dev, "Spk pa info is not a multiple of %d.\n",
			CELL_NUMBER);
		return -EINVAL;
	}

	spk_cnt = elem_cnt / CELL_NUMBER;
	if (spk_cnt > BOARD_FUNC_MAX) {
		dev_warn(dev, "Speaker count %d is greater than %d!\n",
			 spk_cnt, BOARD_FUNC_MAX);
		spk_cnt = BOARD_FUNC_MAX;
	}

	spin_lock_init(&hook_spk_priv.lock);

	buf = devm_kmalloc(dev, elem_cnt * sizeof(u32), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	
	ret = of_property_read_u32_array(np, prop_pa_info, buf, elem_cnt);
	if (ret < 0) {
		dev_err(dev, "Read property '%s' failed!\n", prop_pa_info);
		//return ret;
	}
	for (i = 0; i < spk_cnt; i++) {
		int num = i * CELL_NUMBER;

		/* Get the ctrl type */
		ext_ctrl_type = buf[CELL_CTRL_TYPE + num];
		if (ext_ctrl_type >= BOARD_FUNC_MAX) {
			dev_err(dev, "Ext ctrl type %d is invalid!\n",
				ext_ctrl_type);
			return -EINVAL;
		}

		/* Get the selection of hook function */
		hook_sel = buf[CELL_HOOK + num];
		if (hook_sel >= ARRAY_SIZE(ext_hook_arr)) {
			dev_err(dev,
				"Hook selection %d is invalid!\n", hook_sel);
			return -EINVAL;
		}
		ext_hook->ext_ctrl[ext_ctrl_type] = ext_hook_arr[hook_sel].hook;

		/* Get the private data */
		priv_data = buf[CELL_PRIV + num];
		hook_spk_priv.priv_data[ext_ctrl_type] = priv_data;
		/* Process the shared gpio */
		share_gpio = buf[CELL_SHARE_GPIO + num];
		if (share_gpio > 0) {
			if (share_gpio > spk_cnt) {
				dev_err(dev, "share_gpio %d is bigger than spk_cnt!\n",
					share_gpio);
				ext_hook->ext_ctrl[ext_ctrl_type] = NULL;
				return -EINVAL;
			}
			hook_spk_priv.gpio[ext_ctrl_type] =
				hook_spk_priv.gpio[share_gpio - 1];
			continue;
		}

		ret = of_get_named_gpio_flags(np, prop_pa_gpio, i, NULL);
		if (ret < 0) {
			dev_err(dev, "Get gpio failed:%d!\n", ret);
			ext_hook->ext_ctrl[ext_ctrl_type] = NULL;
			return ret;
		}
		hook_spk_priv.gpio[ext_ctrl_type] = ret;

		pr_info("ext_ctrl_type %d hook_sel %d priv_data %d gpio %d",
			ext_ctrl_type, hook_sel, priv_data, ret);

		gpio_flag = GPIOF_DIR_OUT | GPIOF_INIT_LOW ;
		ret = gpio_request_one(hook_spk_priv.gpio[ext_ctrl_type],
				       gpio_flag, NULL);
		if (ret < 0) {
			dev_err(dev, "Gpio request[%d] failed:%d!\n",
				ext_ctrl_type, ret);
			ext_hook->ext_ctrl[ext_ctrl_type] = NULL;
			return ret;
		}
	}
	// add by Foursemi 20210401 start
	adp_gpio = of_get_named_gpio_flags(np, prop_pa_adp_gpio, 0, NULL);
	if (adp_gpio < 0) {
		dev_err(dev, "%s not exit! skip\n", prop_pa_adp_gpio);
	} else {
		ret = gpio_request_one(adp_gpio, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "adp_gpio");
		if (ret < 0) {
			dev_err(dev, "adp_gpio request failed:%d!\n", ret);
			return ret;
		}
	}
// add by Foursemi 20210401 end
#ifdef FS_AW_PA
	// add by Foursemi 20210426 start
	if(!pa_is_aw){
	spin_lock_irqsave(&hook_spk_priv.lock, gpio_flag);

	gpio_set_value(hook_spk_priv.gpio[2], 0);
	udelay(20);
	gpio_set_value(hook_spk_priv.gpio[2], 1);
	udelay(500);
	gpio_set_value(hook_spk_priv.gpio[2], 0);

	gpio_set_value(hook_spk_priv.gpio[0], 0);
	udelay(20);
	gpio_set_value(hook_spk_priv.gpio[0], 1);
	udelay(500);
	gpio_set_value(hook_spk_priv.gpio[0], 0);

	spin_unlock_irqrestore(&hook_spk_priv.lock, gpio_flag);
	}
	// add by Foursemi 20210426 end
#endif

	return 0;
}
int sprd_asoc_card_parse_ext_hook(struct device *dev,
				  struct sprd_asoc_ext_hook *ext_hook)
{
	ext_debug_sysfs_init();
	return sprd_asoc_card_parse_hook(dev, ext_hook);
}

MODULE_ALIAS("platform:asoc-sprd-card");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASoC SPRD Sound Card Utils - Hooks");
MODULE_AUTHOR("Peng Lee <peng.lee@spreadtrum.com>");
