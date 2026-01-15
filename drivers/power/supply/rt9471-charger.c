
#include "rt9471-charger.h"

struct mutex rt9471_lock;

static const u32 rt9471_vac_ovp[] = {
	5800000, 6500000, 10900000, 14000000,
};

static const u32 rt9471_wdt[] = {
	0, 40, 80, 160,
};

static const u32 rt9471_otgcc[] = {
	500000, 1200000,
};

static int rt9471_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	u8 cmd_buf[2] = {reg, data};
	struct i2c_msg msg;
	int ret = 0;

	memset(&msg, 0, sizeof(struct i2c_msg));

	msg.addr = RT9471_SLAVE_ADDR;
	msg.flags = 0;
	msg.buf = cmd_buf;
	msg.len = sizeof(cmd_buf);

	mutex_lock(&rt9471_lock);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
	{
		printk("[%s] i2c transfer failed, ret:%d\n", __func__, ret);
	}
	mutex_unlock(&rt9471_lock);

	return ret;
}

static int rt9471_i2c_read_byte(struct i2c_client *client, u8 reg, u8 *data)		//id should be 0x70
{
	int ret = 0;
	struct i2c_msg msg[2];

	memset(msg, 0, 2 * sizeof(struct i2c_msg));

	msg[0].addr = RT9471_SLAVE_ADDR;
	msg[0].flags = 0;
	msg[0].len = sizeof(reg);
	msg[0].buf = &reg;

	msg[1].addr = RT9471_SLAVE_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	mutex_lock(&rt9471_lock);
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
	{
		printk("[%s] i2c transfer failed, ret:%d\n", __func__, ret);
	}
	mutex_unlock(&rt9471_lock);

	return ret;
}

static inline u8 rt9471_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if (target < min)
		return 0;

	if (target >= max)
		target = max;

	return (target - min) / step;
}

static inline u8 rt9471_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
					    u32 target)
{
	u32 i = 0;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static inline u32 rt9471_closest_value(u32 min, u32 max, u32 step, u8 regval)
{
	u32 val = 0;

	val = min + regval * step;
	if (val > max)
		val = max;

	return val;
}

static int rt9471_i2c_update_bits(struct i2c_client *client, u8 cmd, u8 data,
				  u8 mask)
{
	int ret = 0;
	u8 regval = 0;

	//mutex_lock(&chip->io_lock);
	ret = rt9471_i2c_read_byte(client, cmd, &regval);
	if (ret < 0)
		goto out;

	regval &= ~mask;
	regval |= (data & mask);

	ret = rt9471_i2c_write_byte(client, cmd, regval);
out:
	//mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int rt9471_set_bit(struct i2c_client *client, u8 cmd, u8 mask)
{
	return rt9471_i2c_update_bits(client, cmd, mask, mask);
}

static inline int rt9471_clr_bit(struct i2c_client *client, u8 cmd, u8 mask)
{
	return rt9471_i2c_update_bits(client, cmd, 0x00, mask);
}

int rt9471_dump_register(struct i2c_client *client)
{
	int i;
	int ret;
	u8 value = 0;

	pr_info("[rt9471]\n");
	for (i = 0; i < RT9471_REG_NUM; i++) {
		ret = rt9471_i2c_read_byte(client, i, &value);
		if (ret < 0) {
			pr_info("[rt9471] i2c transfor error\n");
			return ret;
		}
		pr_info("[0x%x]=0x%x\n", i, value);
	}

	return 0;
}

int rt9471_enable_jeita(struct i2c_client *client, bool en)
{
	pr_info("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(client, RT9471_REG_JEITA, RT9471_JEITA_EN_MASK);
}

int rt9471_read_chip_id(struct i2c_client *client, u8 *chip_id)
{
	unsigned char value = 0;
	int ret;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_INFO, &value);
	if (ret < 0)
	{
		printk("rt9471_read_chip_id failed\n");
		return ret;
	}

	*chip_id = (value & RT9471_DEVID_MASK) >> RT9471_DEVID_SHIFT;

	return ret;
}

int rt9471_reset_register(struct i2c_client *client)
{
	int ret = 0;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_INFO,
		RT9471_REGRST_MASK, RT9471_REGRST_MASK);
	if (ret < 0)
	{
		printk("rt9471_reset_register failed\n");
		return ret;
	}

	return ret;
}

int rt9471_set_vac_ovp(struct i2c_client *client, u32 vac_ovp)
{
	u8 regval = 0;

	regval = rt9471_closest_reg_via_tbl(rt9471_vac_ovp,
					    ARRAY_SIZE(rt9471_vac_ovp),
					    vac_ovp);

	printk("%s vac_ovp = %d(0x%02X)\n", __func__, vac_ovp, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_VBUS,
				      regval << RT9471_VAC_OVP_SHIFT,
				      RT9471_VAC_OVP_MASK);
}

int rt9471_set_mivr(struct i2c_client *client, u32 mivr)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_MIVR_MIN, RT9471_MIVR_MAX,
				    RT9471_MIVR_STEP, mivr);

	printk("%s mivr = %d(0x%02X)\n", __func__, mivr, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_VBUS,
				      regval << RT9471_MIVR_SHIFT,
				      RT9471_MIVR_MASK);
}

int rt9471_set_cv(struct i2c_client *client, u32 cv)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_CV_MIN, RT9471_CV_MAX,
				    RT9471_CV_STEP, cv);

	printk("%s cv = %d(0x%02X)\n", __func__, cv, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_VCHG,
				      regval << RT9471_CV_SHIFT,
				      RT9471_CV_MASK);
}

int rt9471_set_ieoc(struct i2c_client *client, u32 ieoc)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_IEOC_MIN, RT9471_IEOC_MAX,
				    RT9471_IEOC_STEP, ieoc);

	printk("%s ieoc = %d(0x%02X)\n", __func__, ieoc, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_EOC,
				      regval << RT9471_IEOC_SHIFT,
				      RT9471_IEOC_MASK);
}

int rt9471_set_aicr(struct i2c_client *client, u32 aicr)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_AICR_MIN, RT9471_AICR_MAX,
				    RT9471_AICR_STEP, aicr);
	/* 0 & 1 are both 50mA */
	if (aicr < RT9471_AICR_MAX)
		regval += 1;

	printk("%s aicr = %d(0x%02X)\n", __func__, aicr, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_IBUS,
				      regval << RT9471_AICR_SHIFT,
				      RT9471_AICR_MASK);
}

int rt9471_set_wdt(struct i2c_client *client, u32 sec)
{
	u8 regval = 0;

	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= 40 && sec > 0)
		sec = 40;
	regval = rt9471_closest_reg_via_tbl(rt9471_wdt, ARRAY_SIZE(rt9471_wdt),
					    sec);

	printk("%s time = %d(0x%02X)\n", __func__, sec, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_TOP,
				      regval << RT9471_WDT_SHIFT,
				      RT9471_WDT_MASK);
}

int rt9471_enable_safe_tmr(struct i2c_client *client, bool en)
{
	printk("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(client, RT9471_REG_CHGTIMER, RT9471_SAFETMR_EN_MASK);
}

int rt9471_enable_qon_rst(struct i2c_client *client, bool en)
{
	printk("%s en = %d\n", __func__, en);
	return (en ? rt9471_set_bit : rt9471_clr_bit)
		(client, RT9471_REG_TOP, RT9471_QONRST_MASK);
}

int rt9471_is_batfet_on(struct i2c_client *client)
{
	u8 reg_val = 0;
	int ret;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_FUNCTION, &reg_val);
	if (ret < 0)
	{
		printk("rt9471_is_batfet_on failed\n");
		return ret;
	}

	reg_val &= RT9471_BATFETDIS_MASK;

	if(reg_val)
		return 0;
	else
		return 1;
}

int rt9471_is_chg_enabled(struct i2c_client *client)
{
	u8 reg_val = 0;
	int ret;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_FUNCTION, &reg_val);
	if (ret < 0)
	{
		printk("rt9471_is_chg_enabled failed\n");
		return ret;
	}
	reg_val &= RT9471_CHG_EN_MASK;

	if(reg_val)
		return 1;
	else
		return 0;
}

int rt9471_enable_shipmode(struct i2c_client *client)
{
	const u8 mask = RT9471_BATFETDIS_MASK;

	printk("%s\n", __func__);

	return rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION, mask, mask);
}

int rt9471_is_qon_rst_enable(struct i2c_client *client)
{
	u8 reg_val = 0;
	int ret;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_TOP, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= (1 << 7);

	if(reg_val)
	{
		printk("RT9471_charger_is_rst_enable:Enable BATFET reset.\n");
		return 1;
	}
	else
	{
		printk("RT9471_charger_is_rst_enable:Disable BATFET reset.\n");
		return 0;
	}
}

int rt9471_disable_shipmode(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
				  0, RT9471_BATFETDIS_MASK);
	if (ret < 0)
	{
		printk("rt9471_disable_shipmode failed\n");
		return ret;
	}

	return ret;
}

int rt9471_enable_chg(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
		0x01 << RT9471_CHG_EN_SHIFT, RT9471_CHG_EN_MASK);

	if (ret < 0)
	{
		printk("rt9471_enable_chg failed\n");
		return ret;
	}

   return ret;
}

int rt9471_disable_chg(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
		0x00 << RT9471_CHG_EN_SHIFT, RT9471_CHG_EN_MASK);

   if (ret < 0)
	{
		printk("rt9471_disable_chg failed\n");
		return ret;
	}

   return ret;
}

int rt9471_disable_hiz_mode(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
		0, RT9471_HZ_MASK);

	if (ret < 0)
	{
		printk("rt9471_disable_hiz_mode failed\n");
		return ret;
	}

   return ret;
}

int rt9471_enable_hiz_mode(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
		0x01 << RT9471_HZ_SHIFT, RT9471_HZ_MASK);

		if (ret < 0)
	{
		printk("rt9471_enable_hiz_mode failed\n");
		return ret;
	}

   return ret;
}

int rt9471_set_ichg(struct i2c_client *client, u32 ichg)
{
	u8 regval = 0;

	regval = rt9471_closest_reg(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
				    RT9471_ICHG_STEP, ichg);

	printk("%s ichg = %d(0x%02X)\n", __func__, ichg, regval);

	return rt9471_i2c_update_bits(client, RT9471_REG_ICHG,
				      regval << RT9471_ICHG_SHIFT,
				      RT9471_ICHG_MASK);
}

int rt9471_get_ichg(struct i2c_client *client, u32 *ichg)
{
	int ret = 0;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_ICHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_ICHG_MASK) >> RT9471_ICHG_SHIFT;
	*ichg = rt9471_closest_value(RT9471_ICHG_MIN, RT9471_ICHG_MAX,
				  RT9471_ICHG_STEP, regval);

	return ret;
}

int rt9471_get_aicr(struct i2c_client *client, u32 *aicr)
{
	int ret = 0;
	u8 regval = 0;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_IBUS, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & RT9471_AICR_MASK) >> RT9471_AICR_SHIFT;
	*aicr = rt9471_closest_value(RT9471_AICR_MIN, RT9471_AICR_MAX,
					  RT9471_AICR_STEP, regval);
	if (*aicr > RT9471_AICR_MIN && *aicr < RT9471_AICR_MAX)
		 *aicr -= RT9471_AICR_STEP;
 
	return ret;
}

int rt9471_kick_wdt(struct i2c_client *client)
{
	printk("%s\n", __func__);
	return rt9471_set_bit(client, RT9471_REG_TOP, RT9471_WDTCNTRST_MASK);
}

bool rt9471_is_otg_en(struct i2c_client *client)
{
	int ret;
	u8 value = 0;
	bool status = false;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_FUNCTION, &value);
	if (ret) {
		printk("get RT9471 charger otg valid status failed\n");
		return status;
	}

	if (value & RT9471_OTG_EN_MASK)
		status = true;
	else
		printk("otg is not valid, REG_1 = 0x%x\n", value);

	return status;
}

bool rt9471_is_otg_fault(struct i2c_client *client)
{
	int ret;
	u8 value = 0;
	bool status = false;

	ret = rt9471_i2c_read_byte(client, RT9471_REG_IRQ3, &value);
	if (ret) {
		printk("rt9471_is_otg_fault failed\n");
		return status;
	}

	if (value & 0x01)
		status = true;
	else
		printk("otg is not fault, REG_1 = 0x%x\n", value);

	return status;
}

int rt9471_enable_otg(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
		1 << RT9471_OTG_EN_SHIFT, RT9471_OTG_EN_MASK);
	if (ret < 0) {
		printk("enable RT9471 otg failed\n");
	}

	return ret;
}

int rt9471_disable_otg(struct i2c_client *client)
{
	int ret;

	ret = rt9471_i2c_update_bits(client, RT9471_REG_FUNCTION,
		0 << RT9471_OTG_EN_SHIFT, RT9471_OTG_EN_MASK);
	if (ret < 0) {
		printk("diaable RT9471 otg failed\n");
	}

	return ret;

}

int rt9471_set_otgcc(struct i2c_client *client, u32 cc)
{
	printk("%s cc = %d\n", __func__, cc);
	return (cc <= rt9471_otgcc[0] ? rt9471_clr_bit : rt9471_set_bit)
		(client, RT9471_REG_OTGCFG, RT9471_OTGCC_MASK);
}


