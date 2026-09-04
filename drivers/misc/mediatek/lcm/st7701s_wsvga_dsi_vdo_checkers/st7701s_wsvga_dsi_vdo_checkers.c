/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#include <upmu_common.h>

/*mt8163 read auxadc*/
#include "../../auxadc/mtk_auxadc.h"
#include "lcm_drv.h"
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (960)
#define LCM_PHYSICAL_WIDTH_MM	(63)	// Screen width in MM
#define LCM_PHYSICAL_HEIGHT_MM	(125)	// Screen height in MM
#define LCM_PHYSICAL_WIDTH_UM	(LCM_PHYSICAL_WIDTH_MM * 1000)	// Screen width in UM
#define LCM_PHYSICAL_HEIGHT_UM	(LCM_PHYSICAL_HEIGHT_MM * 1000)	// Screen height in UM
#define REGFLAG_DELAY 0xFE
/* END OF REGISTERS MARKER */
#define REGFLAG_END_OF_TABLE 0xFC
#define ST_KD_CTP	0x1	/* KD, using ST7701S IC, CTP cell */
#define ST_INX		0x2	/* INX, using ST7701S IC */
#define ST_TRULY	0x3	/* TRULY, using ST7701S IC */
#define ST_PROTOKD	0x0	/* PROTOKD, using ST7701S IC */
#define ST_KD_INX	0x4	/* Update in EVT, INX cell */
#define ST_KD_HSD	0x5	/* Update in EVT, HSD cell */

#define ST_UNKNOWN	0xFF

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
#define GPIO_OUT_HIGH 1
#define GPIO_OUT_LOW 0
#define TRUE 1
#define FALSE 0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static struct LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.set_gpio_out = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};

#define SET_RESET_PIN(v)      					(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	        			(lcm_util.set_gpio_out((n), (v)))
#define UDELAY(n) 						(lcm_util.udelay(n))
#define MDELAY(n) 						(lcm_util.mdelay(n))

static int lcm_auxadc_channel = 0;
static unsigned char vendor_id;
static unsigned char lcm_id;

static struct regulator *lcm_vgp;
static struct regulator *lcm_vio;

static unsigned int GPIO_LCD_RST_EN;
/* Checkers use AUX_IN0(channel 0) to read lcm vendor_id */
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
static void lcm_get_gpio_infor(struct device *dev);
static void lcm_get_adc_channel(struct device *dev);
static int lcm_get_vgp_supply(struct device *dev);
static int lcm_get_vio_supply(struct device *dev);

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE	0
struct LCM_setting_table
{
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

#ifdef BUILD_LK

#ifdef GPIO_LCM_RST
#define GPIO_LCD_RST_EN		GPIO_LCM_RST
#else
#define GPIO_LCD_RST_EN		(0xFFFFFFFF)
#endif
#endif


static void lcm_get_adc_channel(struct device *dev)
{
	pr_debug("LCM: lcm_get_adc_channel is going\n");

	if (of_property_read_u32(dev->of_node, "lcm_auxadc_channel", &lcm_auxadc_channel))
		pr_err("LCM: cannnot read adc channel from dts\n");
	else
		pr_debug("LCM: lcm_auxadc_channel is channel-%d\n", lcm_auxadc_channel);
}

static void lcm_get_gpio_infor(struct device *dev)
{
	pr_debug("LCM: lcm_get_gpio_infor is going\n");

	/* Checkers no external lcm-pmic, no need to asign en pin */
	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "lcm_reset_gpio", 0);
	gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	pr_debug("LCM: GPIO_LCD_RST_EN=%d.\n",GPIO_LCD_RST_EN);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static int lcm_get_vgp_supply(struct device *dev)
{
	int ret = 0;
	struct regulator *lcm_vgp_ldo;	/* Checkers 2V8, use mt6323-VIO28 */

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "lcm-vgp");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get lcm-vgp LDO, %d\n", ret);
		return ret;
	}

	lcm_vgp = lcm_vgp_ldo;
	pr_debug("LCM: lcm get lcm-vgp supply ok.\n");

	return ret;
}

static int lcm_get_vio_supply(struct device *dev)
{
        int ret = 0;
        struct regulator *lcm_vio_ldo;	/* Checkers 1V8, use mt6323-VGP3 */

        pr_debug("LCM: lcm_get_vio_supply is going\n");

        lcm_vio_ldo = devm_regulator_get(dev, "lcm-vio");
        if (IS_ERR(lcm_vio_ldo)) {
                ret = PTR_ERR(lcm_vio_ldo);
                dev_err(dev, "failed to get lcm-vio LDO, %d\n", ret);
                devm_regulator_put(lcm_vio_ldo);
                return ret;
        }

        lcm_vio = lcm_vio_ldo;
        pr_debug("LCM: lcm get lcm-vio supply ok.\n");

        return ret;
}

int lcm_vio_supply_enable(void)
{
	int ret = 0;
	unsigned int volt;

	pr_debug("LCM: Enter lcm_vio_supply_enable\n");

	if (NULL == lcm_vio) {
		pr_err("LCM: Oops, do nothing and leave lcm_vio_supply_enable\n");
		return 0;
	}

	pr_debug("LCM: set regulator voltage lcm_vio voltage to 1.8V\n");
	/* Checkers lcm_vio=1V8, use mt6323-VGP3 */
	ret = regulator_set_voltage(lcm_vio, 1800000, 1800000);

	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vio voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vio);
	if (volt == 1800000)
		pr_debug("LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vio);

	if (ret != 0)
		pr_err("LCM: Failed to enable lcm_vio: %d\n", ret);

	return ret;
}

int lcm_vio_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	pr_debug("LCM: lcm_vio_supply_disable\n");

	if (NULL == lcm_vio) {
		pr_err("LCM: Oops, do nothing and leave lcm_vio_supply_disable\n");
		return 0;
	}

	pr_debug("LCM: close supply lcm_vio(1V8)\n");
	/* Checkers lcm_vio=1V8, use mt6323-VGP3 */
	isenable = regulator_is_enabled(lcm_vio);

	pr_debug("LCM: lcm_vio query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vio);

		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vio: %d\n", ret);
			return ret;
		}
		else {
			pr_debug("LCM: lcm disable lcm_vio\n");
		}

		/* verify */
		isenable = regulator_is_enabled(lcm_vio);
		if (!isenable)
			pr_debug("LCM: lcm_vio regulator disable pass\n");
		else
			pr_err("LCM: lcm_vio regulator disable fail\n");
	}

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret = 0;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp) {
		pr_err("LCM: Oops, do nothing and leave lcm_vgp_supply_enable\n");
		return 0;
	}

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 2.8V\n");
	/* Checkers lcm_vgp=2V8, use mt6323-VIO28, constant voltage */
	ret = regulator_set_voltage(lcm_vgp, 2800000, 2800000);

	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 2800000)
		pr_debug("LCM: check regulator voltage=2800000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=2800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);

	if (ret != 0)
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);

	return ret;
}

int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: close supply lcm_vgp(2V8)\n");
	/* Checkers lcm_vgp=2V8, use mt6323-VIO28, constant voltage */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm_vgp query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);

		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		else {
			pr_debug("LCM: lcm disable lcm_vgp\n");
		}

		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_debug("LCM: lcm_vgp regulator disable pass\n");
		else
			pr_err("LCM: lcm_vgp regulator disable fail\n");
	}

	return ret;
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	pr_debug("LCM: lcm_driver_probe \n");

	lcm_get_vgp_supply(dev);
	lcm_vgp_supply_enable();
	lcm_get_vio_supply(dev);
	lcm_vio_supply_enable();
	lcm_get_gpio_infor(dev);
	lcm_get_adc_channel(dev);

	return 0;
}
static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "lcm,lcm_dts_st7701s_checkers",
		.data = 0,
	}, {
		/* sentinel */
	}
};


MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}



static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
			.name = "lcm_dts_st7701s_checkers",
			.owner = THIS_MODULE,
			.of_match_table = lcm_platform_of_match,
			},
};

static int __init lcm_init(void)
{
	pr_debug("LCM: lcm_init\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_err("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static struct LCM_setting_table lcm_initialization_setting_truly[] =
{
	/* just lightup, need improvement */
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xF7,0x01}},
	{0xC1,2,{0x0B,0x02}},
	{0xC2,2,{0x07,0x02}},
	{0xCC,1,{0x30}},
	{0xB0,16,{0x00,0x03,0x0D,0x0F,0x13,0x07,0x05,0x09,0x08,0x21,0x05,0x12,0x10,0x29,0x30,0x1F}},
	{0xB1,16,{0x00,0x14,0x1A,0x0D,0x11,0x07,0x09,0x08,0x07,0x22,0x05,0x14,0x13,0x27,0x30,0x1F}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x70}},	/* VOP 70 4.9V 75 5V B5 5.8V */
	{0xB2,1,{0x89}},	/* VGH 7V */
	{0xB3,1,{0x80}},
	{0xB5,1,{0x49}},	/* VGL 49 -10V 4C -11V */
	{0xB7,1,{0x89}},
	{0xB8,1,{0x21}},
	{0xB9,1,{0x10}},
	{0xBB,1,{0x03}},
	{0xC1,1,{0x78}},
	{0xC2,1,{0x78}},
	{0xD0,1,{0x88}},
	{REGFLAG_DELAY, 100, { }},
	{0xE0,3,{0x00,0x00,0x02}},
	{0xE1,11,{0x04,0x8C,0x00,0x8C,0x03,0x8C,0x00,0x8C,0x01,0x60,0x60}},
	{0xE2,13,{0x30,0x30,0x60,0x60,0xCB,0x8C,0x00,0x8C,0xCA,0x8C,0x00,0x8C,0x00}},
	{0xE3,4,{0x00,0x00,0x33,0x33}},
	{0xE4,2,{0x44,0x44}},
	{0xE5,16,{0x08,0xCD,0xA0,0x8C,0x0C,0xD1,0xA0,0x8C,0x0A,0xCF,0xA0,0x8C,0x0E,0xD3,0xA0,0x8C}},
	{0xE6,4,{0x00,0x00,0x33,0x33}},
	{0xE7,2,{0x44,0x44}},
	{0xE8,16,{0x07,0xCC,0xA0,0x8C,0x0B,0xD0,0xA0,0x8C,0x09,0xCE,0xA0,0x8C,0x0D,0xD2,0xA0,0x8C}},
	{0xEB,7,{0x00,0x00,0x1E,0x1E,0x88,0x00,0x00}},
	{0XEC,2,{0x00,0x00}},
	{0xED,16,{0x04,0x65,0x7F,0xA2,0xBF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB,0x2A,0xF7,0x56,0x40}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
	{0x11,0,{}},
	{REGFLAG_DELAY, 120, { }},
	{0x29,0,{}},
	{REGFLAG_DELAY, 150, { }},
	{REGFLAG_END_OF_TABLE, 0x00, { }},
};

static struct LCM_setting_table lcm_initialization_setting_kd_ctp[] =
{
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xF7,0x01}},
	{0xC1,2,{0x09,0x02}},
	{0xC6,1,{0x21}},
	{0xC2,2,{0x27,0x03}},
	{0xCC,1,{0x10}},
	{0xB0,16,{0x00,0x09,0x13,0x0C,0x10,0x04,0x01,0x05,0x05,0x1B,0x03,0x12,0x10,0x2B,0x33,0x19}},
	{0xB1,16,{0x00,0x0A,0x0F,0x0D,0x12,0x08,0x00,0x0A,0x09,0x1B,0x05,0x52,0x11,0x2A,0x32,0x19}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x6B}},
	{0xB1,1,{0x27}},
	{0xB2,1,{0x88}},
	{0xB3,1,{0x80}},
	{0xB5,1,{0x49}},
	{0xB7,1,{0x87}},
	{0xB8,1,{0x21}},
	{0xB9,2,{0x00,0x13}},
	{0xC1,1,{0x78}},
	{0xC2,1,{0x78}},
	{0xD0,1,{0x88}},
	{0xEF,6,{0x08,0x08,0x08,0x08,0x3F,0x1F}},
	{REGFLAG_DELAY,100,{ }},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xE0,3,{0x00,0x00,0x02}},
	{0xE1,11,{0x02,0x91,0x00,0x00,0x01,0x91,0x00,0x00,0x00,0x60,0x60}},
	{0xE2,13,{0x30,0x30,0x40,0x40,0xC9,0x91,0x00,0x00,0xC8,0x91,0x00,0x00,0x00}},
	{0xE3,4,{0x00,0x00,0x33,0x33}},
	{0xE4,2,{0x44,0x44}},
	{0xE5,16,{0x06,0xCA,0x4B,0x91,0x08,0xCC,0x4B,0x91,0x0A,0xCE,0x4B,0x91,0x0C,0xD0,0x4B,0x91}},
	{0xE6,4,{0x00,0x00,0x33,0x33}},
	{0xE7,2,{0x44,0x44}},
	{0xE8,16,{0x05,0xC9,0x4B,0x91,0x07,0xCB,0x4B,0x91,0x09,0xCD,0x4B,0x91,0x0B,0xCF,0x4B,0x91}},
	{0xEB,7,{0x02,0x01,0xE4,0xE4,0x88,0x00,0x00}},
	{0xEC,2,{0x00,0x00}},
	{0xED,16,{0xFA,0xB0,0x2F,0xF4,0x65,0x7F,0xFF,0xFF,0xFF,0xFF,0xF7,0x56,0x4F,0xF2,0x0B,0xAF}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
	{0x11,0,{ }},
	{REGFLAG_DELAY,120,{ }},
	{0x29, 0, { }},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, { }},
};

static struct LCM_setting_table lcm_initialization_setting_kd_inx[] =
{
	/* just lightup, need improvement */
	{0xFF,5,{0x77,0x01,0x00,0x00,0x13}},
	{0xEF,1,{0x08}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xF7,0x02}},
	{0xC1,2,{0x0E,0x02}},
	{0xC2,2,{0x07,0x02}},
	{0xCC,1,{0x30}},
	{0xB0,16,{0xC0,0x24,0x23,0x10,0x14,0x08,0x11,0x09,0x08,0x29,0x05,0x12,0x10,0x28,0x2E,0x1F}},
	{0xB1,16,{0xC0,0xE1,0x23,0x0C,0x10,0x05,0x0B,0x07,0x07,0x22,0x04,0x11,0x0F,0x22,0x29,0x1F}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x7D}},
	{0xB1,1,{0x61}},
	{0xB2,1,{0x89}},
	{0xB3,1,{0x80}},
	{0xB5,1,{0x4D}},
	{0xB7,1,{0x87}},
	{0xB8,1,{0x21}},
	{0xB9,1,{0x10}},
	{0xC0,1,{0x87}},
	{0xC1,1,{0x78}},
	{0xC2,1,{0x78}},
	{0xD0,1,{0x88}},
	{0xE0,3,{0x00,0x00,0x02}},
	{0xE1,11,{0x03,0x96,0x00,0x96,0x02,0x96,0x00,0x96,0x00,0x40,0x40}},
	{0xE2,13,{0x33,0x33,0x44,0x44,0xD3,0x96,0xD0,0x96,0xD2,0x96,0xCF,0x96,0x00}},
	{0xE3,4,{0x00,0x00,0x33,0x33}},
	{0xE4,2,{0x44,0x44}},
	{0xE5,16,{0x05,0xD8,0x96,0x96,0x07,0xDA,0x96,0x96,0x09,0xD4,0x96,0x96,0x0B,0xD6,0x96,0x96}},
	{0xE6,4,{0x00,0x00,0x33,0x33}},
	{0xE7,2,{0x44,0x44}},
	{0xE8,16,{0x04,0xD7,0x96,0x96,0x06,0xD9,0x96,0x96,0x08,0xD3,0x96,0x96,0x0A,0xD5,0x96,0x96}},
	{0xEB,7,{0x02,0x00,0x4E,0x4E,0xCC,0x88,0x10}},
	{0xEC,2,{0x40,0x00}},
	{0xED,16,{0xFF,0x0F,0x45,0x67,0xF9,0xF8,0xF2,0x3F,0xF3,0x2F,0x8F,0x9F,0x76,0x54,0xF0,0xFF}},
	{0x11,0, { }},
	{REGFLAG_DELAY, 120, { }},
	{0x29,0, { }},
	{REGFLAG_DELAY, 20, { }},
	{REGFLAG_END_OF_TABLE, 0x00, { }},
};

static struct LCM_setting_table lcm_initialization_setting_kd_hsd[] =
{
	/* just lightup, need improvement */
	{0xFF,5,{0x77,0x01,0x00,0x00,0x13}},
	{0xEF,1,{0x08}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xF7,0x01}},
	{0xC1,2,{0x0C,0x02}},
	{0xC2,2,{0x07,0x02}},
	{0xC6,1,{0x21}},
	{0xCC,1,{0x30}},
	{0xB0,16,{0x00,0x0B,0x12,0x0D,0x11,0x07,0x04,0x08,0x07,0x20,0x05,0x12,0x0F,0x23,0x2A,0x1B}},
	{0xB1,16,{0x00,0x0B,0x12,0x0D,0x11,0x05,0x03,0x08,0x08,0x20,0x03,0x10,0x10,0x29,0x30,0x1B}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x57}},
	{0xB1,1,{0x2D}},
	{0xB2,1,{0x87}},
	{0xB3,1,{0x80}},
	{0xB5,1,{0x49}},
	{0xB7,1,{0x87}},
	{0xB8,1,{0x21}},
	{0xB9,1,{0x10}},
	{0xBB,1,{0x00}},
	{0xC0,1,{0x09}},
	{0xC1,1,{0x78}},
	{0xC2,1,{0x78}},
	{0xD0,1,{0x88}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xE0,3,{0x00,0x19,0x00}},
	{0xE1,11,{0x07,0x82,0x00,0x82,0x06,0x82,0x00,0x82,0x00,0x40,0x40}},
	{0xE2,13,{0x30,0x30,0x40,0x40,0xCC,0x82,0x00,0x82,0xCB,0x82,0x00,0x82,0x00}},
	{0xE3,4,{0x00,0x00,0x33,0x33}},
	{0xE4,2,{0x44,0x44}},
	{0xE5,16,{0x09,0xC5,0xAA,0x82,0x0B,0xC7,0xAA,0x82,0x0D,0xC9,0xAA,0x82,0x0F,0xCB,0xAA,0x82}},
	{0xE6,4,{0x00,0x00,0x33,0x33}},
	{0xE7,2,{0x44,0x44}},
	{0xE8,16,{0x08,0xC4,0xAA,0x82,0x0A,0xC6,0xAA,0x82,0x0C,0xC8,0xAA,0x82,0x0E,0xCA,0xAA,0x82}},
	{0xEB,7,{0x00,0x00,0xE4,0xE4,0x88,0x00,0x10}},
	{0xEC,2,{0x3D,0x00}},
	{0xED,16,{0xAB,0x98,0x76,0x54,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x20,0x45,0x67,0x89,0xBA}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
	{0x11,0, { }},
	{REGFLAG_DELAY, 120, { }},
	{0x29,0, { }},
	{REGFLAG_DELAY, 20, { }},
	{REGFLAG_END_OF_TABLE, 0x00, { }},
};

static struct LCM_setting_table lcm_initialization_setting_protokd[] =
{
	{0x11, 0, { }},
	{REGFLAG_DELAY, 120, { }},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xF7,0x01}},
	{0xC1,2,{0x09,0x02}},
	{0xC6,1,{0x21}},
	{0xC2,2,{0x27,0x03}},
	{0xCC,1,{0x10}},
	{0xB0,16,{0x00,0x09,0x13,0x0C,0x10,0x04,0x01,0x05,0x05,0x1B,0x03,0x12,0x10,0x2B,0x33,0x19}},
	{0xB1,16,{0x00,0x0A,0x0F,0x0D,0x12,0x08,0x00,0x0A,0x09,0x1B,0x05,0x12,0x11,0x2A,0x32,0x19}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x68}},
	{0xB1,1,{0x27}},
	{0xB2,1,{0x88}},
	{0xB3,1,{0x80}},
	{0xB5,1,{0x49}},
	{0xB7,1,{0x85}},
	{0xB8,1,{0x10}},
	{0xB9,2,{0x00,0x13}},
	{0xC1,1,{0x78}},
	{0xC2,1,{0x78}},
	{0xD0,1,{0x88}},
	{0xEF,6,{0x08,0x08,0x08,0x08,0x3F,0x1F}},
	{REGFLAG_DELAY,100,{ }},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xE0,3,{0x00,0x00,0x02}},
	{0xE1,11,{0x02,0x91,0x00,0x00,0x01,0x91,0x00,0x00,0x00,0x60,0x60}},
	{0xE2,13,{0x30,0x30,0x40,0x40,0xC9,0x91,0x00,0x00,0xC8,0x91,0x00,0x00,0x00}},
	{0xE3,4,{0x00,0x00,0x33,0x33}},
	{0xE4,2,{0x44,0x44}},
	{0xE5,16,{0x06,0xCA,0x4B,0x91,0x08,0xCC,0x4B,0x91,0x0A,0xCE,0x4B,0x91,0x0C,0xD0,0x4B,0x91}},
	{0xE6,4,{0x00,0x00,0x33,0x33}},
	{0xE7,2,{0x44,0x44}},
	{0xE8,16,{0x05,0xC9,0x4B,0x91,0x07,0xCB,0x4B,0x91,0x09,0xCD,0x4B,0x91,0x0B,0xCF,0x4B,0x91}},
	{0xEB,7,{0x02,0x01,0xE4,0xE4,0x88,0x00,0x00}},
	{0xEC,2,{0x00,0x00}},
	{0xED,16,{0xFA,0xB0,0x2F,0xF4,0x65,0x7F,0xFF,0xFF,0xFF,0xFF,0xF7,0x56,0x4F,0xF2,0x0B,0xAF}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
	{0x29, 0, { }},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, { }},
};

static struct LCM_setting_table lcm_initialization_setting_inx[] =
{
	/* pure_lcm, wait for TDM update */
	{0xFF,5,{0x77,0x01,0x00,0x00,0x13}},
	{0xEF,1,{0x08}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
	{0xC0,2,{0xF7,0x01}},
	{0xC1,2,{0x0E,0x02}},
	{0xC2,2,{0x07,0x02}},
	{0xCC,1,{0x30}},
	{0xB0,16,{0xC0,0x21,0x1F,0x02,0x15,0x07,0x0F,0x0D,0x0A,0x29,0x06,0x15,0x12,0x6C,0x34,0x1F}},
	{0xB1,16,{0xC0,0x11,0x1D,0x1B,0x0F,0x06,0x0A,0x04,0x05,0x23,0x01,0x0D,0x0C,0x27,0x2E,0x1F}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xB0,1,{0x75}},
	{0xB1,1,{0x40}},
	{0xB2,1,{0x89}},
	{0xB3,1,{0x80}},
	{0xB5,1,{0x4D}},
	{0xB7,1,{0x87}},
	{0xB8,1,{0x21}},
	{0xB9,1,{0x10}},
	{0xC0,1,{0x80}},
	{0xC1,1,{0x08}},
	{0xC2,1,{0x08}},
	{0xD0,1,{0x88}},
	{0xE0,3,{0x00,0x00,0x02}},
	{0xE1,11,{0x03,0x96,0x00,0x96,0x02,0x96,0x00,0x96,0x00,0x40,0x40}},
	{0xE2,13,{0x33,0x33,0x44,0x44,0xD3,0x96,0xD0,0x96,0xD2,0x96,0xCF,0x96,0x00}},
	{0xE3,4,{0x00,0x00,0x33,0x33}},
	{0xE4,2,{0x44,0x44}},
	{0xE5,16,{0x05,0xD8,0x96,0x96,0x07,0xDA,0x96,0x96,0x09,0xD4,0x96,0x96,0x0B,0xD6,0x96,0x96}},
	{0xE6,4,{0x00,0x00,0x33,0x33}},
	{0xE7,2,{0x44,0x44}},
	{0xE8,16,{0x04,0xD7,0x96,0x96,0x06,0xD9,0x96,0x96,0x08,0xD3,0x96,0x96,0x0A,0xD5,0x96,0x96}},
	{0xEB,7,{0x02,0x00,0x4E,0x4E,0xCC,0x88,0x10}},
	{0xEC,2,{0x40,0x00}},
	{0xED,16,{0xFF,0x0F,0x45,0x67,0xF9,0xF8,0xF2,0x3F,0xF3,0x2F,0x8F,0x9F,0x76,0x54,0xF0,0xFF}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
	{0x11, 0, { }},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, { }},
	{REGFLAG_END_OF_TABLE, 0x00, { }},
};

static struct LCM_setting_table lcm_suspend_setting[] =
{
	{0x28,  0 , { }},
	{REGFLAG_DELAY, 20, { }},
	{0x10, 0 , { }},
	{REGFLAG_DELAY, 120, { }},
	{REGFLAG_END_OF_TABLE, 0x00, { }}
};

static void lcm_get_st_truly_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH_MM;
	params->physical_height = LCM_PHYSICAL_HEIGHT_MM;
	params->physical_width_um = LCM_PHYSICAL_WIDTH_UM;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT_UM;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode			= BURST_VDO_MODE;
	/* Command mode setting */
	/* 1 Three lane or Four lane */
	params->dsi.LANE_NUM			= LCM_TWO_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num	= 0;

	params->dsi.PS				= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count			= FRAME_WIDTH*3;

	params->dsi.vertical_sync_active	= 6;
	params->dsi.vertical_backporch		= 20;
	params->dsi.vertical_frontporch		= 20;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 6;
	params->dsi.horizontal_backporch	= 20;
	params->dsi.horizontal_frontporch	= 20;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/* PLL CLOCK 1006 X 526 X 24 X 60 % 2 X 1.1 % 2 = 209 */
	params->dsi.PLL_CLOCK	= 209;
	params->dsi.ssc_disable	= 1;	//1:disable ssc , 0:enable ssc
}

static void lcm_get_st_kd_ctp_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH_MM;
	params->physical_height = LCM_PHYSICAL_HEIGHT_MM;
	params->physical_width_um = LCM_PHYSICAL_WIDTH_UM;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT_UM;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode			= BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_TWO_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	// Video mode setting
	params->dsi.intermediat_buffer_num	= 0;

	params->dsi.PS				= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count			= FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock			= 1;
	params->dsi.clk_lp_per_line_enable	= 1;

	params->dsi.vertical_sync_active	= 6;
	params->dsi.vertical_backporch		= 16;
	params->dsi.vertical_frontporch		= 20;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 6;
	params->dsi.horizontal_backporch	= 20;
	params->dsi.horizontal_frontporch	= 30;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/* PLL_CLOCK: 1002 X 536 X 24 X 60 % 2 X 1.1 % 2 = 212 */
	params->dsi.PLL_CLOCK	= 212;
	params->dsi.ssc_disable	= 1; /* 1:disable ssc , 0:enable ssc */
}

static void lcm_get_st_kd_inx_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH_MM;
	params->physical_height = LCM_PHYSICAL_HEIGHT_MM;
	params->physical_width_um = LCM_PHYSICAL_WIDTH_UM;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT_UM;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode			= BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_TWO_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	// Video mode setting
	params->dsi.intermediat_buffer_num	= 0;

	params->dsi.PS				= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count			= FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock			= 1;
	params->dsi.clk_lp_per_line_enable	= 1;

	params->dsi.vertical_sync_active	= 4;
	params->dsi.vertical_backporch		= 20;
	params->dsi.vertical_frontporch		= 20;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 10;
	params->dsi.horizontal_backporch	= 30;
	params->dsi.horizontal_frontporch	= 32;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/* PLL_CLOCK: 1002 X 536 X 24 X 60 % 2 X 1.1 % 2 = 212 */
	params->dsi.PLL_CLOCK	= 220;
	params->dsi.ssc_disable	= 1; /* 1:disable ssc , 0:enable ssc */
}

static void lcm_get_st_kd_hsd_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH_MM;
	params->physical_height = LCM_PHYSICAL_HEIGHT_MM;
	params->physical_width_um = LCM_PHYSICAL_WIDTH_UM;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT_UM;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode			= BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_TWO_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	// Video mode setting
	params->dsi.intermediat_buffer_num	= 0;

	params->dsi.PS				= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count			= FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock			= 1;
	params->dsi.clk_lp_per_line_enable	= 1;

	params->dsi.vertical_sync_active	= 6;
	params->dsi.vertical_backporch		= 20;
	params->dsi.vertical_frontporch		= 16;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 6;
	params->dsi.horizontal_backporch	= 30;
	params->dsi.horizontal_frontporch	= 20;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/* PLL_CLOCK: 1002 X 536 X 24 X 60 % 2 X 1.1 % 2 = 212 */
	params->dsi.PLL_CLOCK	= 212;
	params->dsi.ssc_disable	= 1; /* 1:disable ssc , 0:enable ssc */
}

static void lcm_get_st_protokd_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH_MM;
	params->physical_height = LCM_PHYSICAL_HEIGHT_MM;
	params->physical_width_um = LCM_PHYSICAL_WIDTH_UM;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT_UM;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode			= BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_TWO_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	// Video mode setting
	params->dsi.intermediat_buffer_num	= 0;

	params->dsi.PS				= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count			= FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock			= 1;
	params->dsi.clk_lp_per_line_enable	= 1;

	params->dsi.vertical_sync_active	= 6;
	params->dsi.vertical_backporch		= 6;
	params->dsi.vertical_frontporch		= 6;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 10;
	params->dsi.horizontal_backporch	= 12;
	params->dsi.horizontal_frontporch	= 50;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/* PLL_CLOCK: 978 X 552 X 24 X 60 % 2 X 1.1 % 2 = 213 */
	params->dsi.PLL_CLOCK	= 213;
	params->dsi.ssc_disable	= 1; /* 1:disable ssc , 0:enable ssc */
}

static void lcm_get_st_inx_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH_MM;
	params->physical_height = LCM_PHYSICAL_HEIGHT_MM;
	params->physical_width_um = LCM_PHYSICAL_WIDTH_UM;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT_UM;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode			= BURST_VDO_MODE;

	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_TWO_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size	= 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num	= 0;

	params->dsi.PS				= LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count			= FRAME_WIDTH*3;

	params->dsi.vertical_sync_active	= 6;
	params->dsi.vertical_backporch		= 16;
	params->dsi.vertical_frontporch		= 20;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 6;
	params->dsi.horizontal_backporch	= 20;
	params->dsi.horizontal_frontporch	= 30;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	/* PLL_CLOCK: 1002 X 536 X 24 X 60 % 2 X 1.1 % 2 = 212 */
	params->dsi.PLL_CLOCK	= 212;
	params->dsi.ssc_disable	= 1; /* 1:disable ssc , 0:enable ssc */
}

static void lcm_reset(void)
{
	pr_info("[KERNEL/LCM] %s, - Reset\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_HIGH);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_LOW);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_HIGH);
	MDELAY(120);
}

static char *lcm_get_vendor_type(void)
{
	switch (vendor_id) {
	case ST_KD_CTP: return "ST_KD_CTP\0";
	case ST_INX: return "ST_INX\0";
	case ST_TRULY: return "ST_TRULY\0";
	case ST_PROTOKD: return "ST_PROTOKD\0";
	case ST_KD_INX: return "ST_KD_INX\0";
	case ST_KD_HSD: return "ST_KD_HSD\0";
	default: return "UNKNOWN\0";
	}
}

static void init_checkers_st7701s_truly_lcm(void)
{
	push_table(lcm_initialization_setting_truly, sizeof(lcm_initialization_setting_truly) / sizeof(struct LCM_setting_table), 1);
}

/* vendor suggest proto and hvt KD lcm can share the same initial code */
static void init_checkers_st7701s_protokd_lcm(void)
{
	push_table(lcm_initialization_setting_protokd, sizeof(lcm_initialization_setting_protokd) / sizeof(struct LCM_setting_table), 1);
}

static void init_checkers_st7701s_kd_ctp_lcm(void)
{
	push_table(lcm_initialization_setting_kd_ctp, sizeof(lcm_initialization_setting_kd_ctp) / sizeof(struct LCM_setting_table), 1);
}

static void init_checkers_st7701s_kd_inx_lcm(void)
{
	push_table(lcm_initialization_setting_kd_inx, sizeof(lcm_initialization_setting_kd_inx) / sizeof(struct LCM_setting_table), 1);
}

static void init_checkers_st7701s_kd_hsd_lcm(void)
{
	push_table(lcm_initialization_setting_kd_hsd, sizeof(lcm_initialization_setting_kd_hsd) / sizeof(struct LCM_setting_table), 1);
}

static void init_checkers_st7701s_inx_lcm(void)
{
	push_table(lcm_initialization_setting_inx, sizeof(lcm_initialization_setting_inx) / sizeof(struct LCM_setting_table), 1);
}

static int __init setup_lcm_id(char *str)
{
	int id;

	if (get_option(&str, &id))
	{
		lcm_id = (unsigned char)id;
	}
	return 0;
}
__setup("nt35521_id=", setup_lcm_id);

static void get_lcm_id(void)
{
	vendor_id = (unsigned int)lcm_id;
}

static void lcm_init_lcm(void)
{
	get_lcm_id();

	if (vendor_id == ST_TRULY)
		init_checkers_st7701s_truly_lcm();		/* ST7701S TRULY panel */
	else if (vendor_id == ST_KD_CTP)
		init_checkers_st7701s_kd_ctp_lcm(); 		/* ST7701S KD panel, CTP glass */
	else if (vendor_id == ST_INX)
		init_checkers_st7701s_inx_lcm(); 		/* ST7701S INX panel */
	else if (vendor_id == ST_PROTOKD)
		init_checkers_st7701s_protokd_lcm(); 		/* ST7701S PROTOKD panel */
	else if (vendor_id == ST_KD_INX)
		init_checkers_st7701s_kd_inx_lcm();		/* ST7701S KD panel, INX glass */
	else if (vendor_id == ST_KD_HSD)
		init_checkers_st7701s_kd_hsd_lcm();		/* ST7701S KD panel, HSD glass */
	else
		init_checkers_st7701s_kd_ctp_lcm();			/* Default ST7701S KD panel, CTP glass */

	pr_debug("[ST7701S_lcm]lcm_init_lcm func:Kernel Checkers st7701s %s lcm init ok!\n",lcm_get_vendor_type());
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend_power(void)
{
	pr_debug("LCM: lcm try to turn off supply\n");

	if (lcm_vio_supply_disable())
		pr_err(KERN_ERR "<%s>ST7701S lcm(lcm_vio) failed to turn off supply", __func__);

	if (lcm_vgp_supply_disable())
		pr_err(KERN_ERR "<%s>ST7701S lcm(lcm_vgp) failed to turn off supply", __func__);

}

static void lcm_resume_power(void)
{
	pr_debug("LCM: lcm try to turn on supply\n");

	if(lcm_vgp_supply_enable())
		pr_err(KERN_ERR "<%s>ST7701S lcm(lcm_vgp) failed to turn on supply", __func__);

	if(lcm_vio_supply_enable())
		pr_err(KERN_ERR "<%s>ST7701S lcm(lcm_vio) failed to turn on supply", __func__);

}

static void lcm_resume(void)
{
	unsigned char buf[4];
	get_lcm_id();

	MDELAY(10);
	lcm_reset();

	if (vendor_id == ST_TRULY)
		init_checkers_st7701s_truly_lcm(); 		/* ST7701S TRULY panel */
	else if (vendor_id == ST_KD_CTP)
		init_checkers_st7701s_kd_ctp_lcm();			/* ST7701S KD panel, CTP glass*/
	else if (vendor_id == ST_INX)
		init_checkers_st7701s_inx_lcm();		/* ST7701S INX panel */
	else if (vendor_id == ST_PROTOKD)
		init_checkers_st7701s_protokd_lcm();		/* ST7701S PROTOKD panel */
	else if (vendor_id == ST_KD_INX)
		init_checkers_st7701s_kd_inx_lcm();		/* ST7701S KD panel, INX glass */
	else if (vendor_id == ST_KD_HSD)
		init_checkers_st7701s_kd_hsd_lcm();		/* ST7701S KD panel, HSD glass */
	else
		init_checkers_st7701s_kd_ctp_lcm();			/* Default ST7701S KD panel, CTP glass */

	read_reg_v2(0x0A, &buf[0], 1);	/* ESD: read 1 bytes from 0Ah */
	pr_debug("[KER/LCM] %s, ESD:0x%x\n", __func__, buf[0]);
}


struct LCM_DRIVER st7701s_wsvga_dsi_vdo_checkers_st_truly_lcm_drv = {
	.name           = "st7701s_wsvga_dsi_vdo_checkers_st_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_st_truly_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};


struct LCM_DRIVER st7701s_wsvga_dsi_vdo_checkers_st_kd_ctp_lcm_drv = {
	.name           = "st7701s_wsvga_dsi_vdo_checkers_st_kd_ctp",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_st_kd_ctp_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};

struct LCM_DRIVER st7701s_wsvga_dsi_vdo_checkers_st_inx_lcm_drv = {
	.name           = "st7701s_wsvga_dsi_vdo_checkers_st_inx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_st_inx_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};

struct LCM_DRIVER st7701s_wsvga_dsi_vdo_checkers_st_protokd_lcm_drv = {
	.name           = "st7701s_wsvga_dsi_vdo_checkers_st_protokd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_st_protokd_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};

struct LCM_DRIVER st7701s_wsvga_dsi_vdo_checkers_st_kd_inx_lcm_drv = {
	.name           = "st7701s_wsvga_dsi_vdo_checkers_st_kd_inx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_st_kd_inx_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};

struct LCM_DRIVER st7701s_wsvga_dsi_vdo_checkers_st_kd_hsd_lcm_drv = {
	.name           = "st7701s_wsvga_dsi_vdo_checkers_st_kd_hsd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_st_kd_hsd_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};

