#ifndef __LINUX_SC2730_USB_CHARGER_INCLUDED
#define __LINUX_SC2730_USB_CHARGER_INCLUDED

#include <linux/delay.h>
#include <linux/regmap.h>
#include <uapi/linux/usb/charger.h>

#define SC2730_CHARGE_STATUS		0x1b9c
#define BIT_CHG_DET_DONE		BIT(11)
#define BIT_SDP_INT			BIT(7)
#define BIT_DCP_INT			BIT(6)
#define BIT_CDP_INT			BIT(5)
extern int pogo_charger_is_pogo_charging(void);
extern bool typec_int_status;

static enum usb_charger_type sc27xx_charger_detect(struct regmap *regmap)
{
	enum usb_charger_type type;
	u32 status = 0, val;
	int ret, cnt = 40;

	do {
		ret = regmap_read(regmap, SC2730_CHARGE_STATUS, &val);
		if (ret)
			return UNKNOWN_TYPE;

		if (val & BIT_CHG_DET_DONE) {
			status = val & (BIT_CDP_INT | BIT_DCP_INT | BIT_SDP_INT);
			break;
		}

		msleep(50);
	} while (--cnt > 0);

	switch (status) {
	case BIT_CDP_INT:
		type = CDP_TYPE;
		break;
	case BIT_DCP_INT:
		type = DCP_TYPE;
		if(typec_int_status==0){
			printk("==typec_int_status:%d",typec_int_status);
			type=UNKNOWN_TYPE;
		}
		break;
	case BIT_SDP_INT:
		type = SDP_TYPE;
		break;
	default:
	if (1 == pogo_charger_is_pogo_charging())//pogo charger
		type = DCP_TYPE;
	else if (typec_int_status == 0)
		type = FLOAT_TYPE;
	else
		type = UNKNOWN_TYPE;
	}

	return type;
}

#endif
