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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <asm/atomic.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <misc/gating.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...) pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  pr_debug(PFX  fmt, ##args)

#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

struct cam_gating {
	struct gpio_desc *cam_pwr_en;
	enum gating_state cur_state;
	enum gating_state req_state;
	struct delayed_work work;
	struct mutex mutex;
};
static struct cam_gating *cam_gate;

/* conf_cam_gate() : configure camera gating */
static enum gating_state conf_cam_gate(enum gating_state state)
{
	int val = !state;

	gpiod_set_value_cansleep(cam_gate->cam_pwr_en, val);

	if (unlikely(gpiod_get_value_cansleep(cam_gate->cam_pwr_en) != val)) {
		val = !val;
		PK_ERR("%s: failed to set state : %d\n", __func__, state);
	}

	cam_gate->cur_state = !val;

	return !val;
}

/* set_gate_state() : check and configure the camera gating state */
static enum gating_state set_gate_state(enum gating_state state)
{
	int latch;

	if (!cam_gate)
		return state;

	latch = gating_state_get();

	mutex_lock(&cam_gate->mutex);

	if (latch == UNGATED || latch == GATED)
		cam_gate->req_state = latch;

	/* Make sure device is in UNGATED Mode before trying to enable CAM */
	if (state == UNGATED && cam_gate->req_state != UNGATED) {
		pr_info("Failed to enable CAM, GATING Mode is ON\n");
		state = GATED;
		goto out;
	}

	if (state != cam_gate->cur_state)
		state = conf_cam_gate(state);
out :
	mutex_unlock(&cam_gate->mutex);
	return state;
}

int cntVCAMD = 0;
int cntVCAMA = 0;
int cntVCAMIO = 0;
int cntVCAMAF = 0;
int cntVCAMD_SUB = 0;
int cntVCAMI2C = 0;

/* #define NEED_MANUAL_VCAMI2C_POWER */

/* GPIO Pin control */
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo0_h = NULL;
struct pinctrl_state *cam_ldo0_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/* Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	} else {
		/* Set pnd default state as low */
		ret = pinctrl_select_state(camctrl, cam0_pnd_l);
		if (ret)
			pr_debug("%s : failed to select cam0_pnd_l state: %d\n", __func__, ret);
	}

	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/* Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/* externel LDO enable */
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}

	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
		case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
		case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
		case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
		default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{
	if (_hwPowerOn(PinIdx, powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMAF)
			cntVCAMAF += 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB += 1;
		else if (powerId == VCAMI2C)
			cntVCAMI2C += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(PinIdx, powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB -= 1;
		else if (powerId == VCAMI2C)
			cntVCAMI2C -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(int PinIdx, char *mode_name)
{

	int i = 0;

	PK_DBG
	("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
		cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(PinIdx, VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(PinIdx, VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(PinIdx, VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(PinIdx, VCAMAF);
	for (i = 0; i < cntVCAMD_SUB; i++)
		_hwPowerDown(PinIdx, SUB_VCAMD);
	for (i = 0; i < cntVCAMI2C; i++)
		_hwPowerDown(PinIdx, VCAMI2C);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;
	cntVCAMI2C = 0;

}


int kdCISModulePowerOn(enum CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
	char *mode_name)
{
	u32 pinSetIdx = 0;

	u32 pinSet[3][8] = {

		{CAMERA_CMRST_PIN,
			CAMERA_CMRST_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			CAMERA_CMPDN_PIN,
			CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		{CAMERA_CMRST1_PIN,
			CAMERA_CMRST1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			CAMERA_CMPDN1_PIN,
			CAMERA_CMPDN1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		{GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		}
	};

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

	if (On) {
		/* Check cam_gate state before powering on sensor */
		if (set_gate_state(UNGATED) != UNGATED) {
			PK_DBG("[CAMERA SENSOR] Falied to UNGATE CAM\n");
			goto _kdCISModulePowerOn_exit_;
		}

        /* VCAM_I2C */
#ifdef NEED_MANUAL_VCAMI2C_POWER
		if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMI2C, VOL_1800, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMI2C), power id = %d \n", VCAMI2C);
			goto _kdCISModulePowerOn_exit_;
		}
#endif

		PK_DBG(" --- SensorName: %s is going to power on... \n",currSensorName);

		if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV9734_MIPI_RAW))) {
			PK_ERR("[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx,currSensorName);

			ISP_MCLK1_EN(1);
			mdelay(2);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, 0);
			mdelay(5);

			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			/* VCAM_A */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(5);

			/* VCAM_D */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1200, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, 1);
		} else {
			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
		}
	} else { /* power OFF */

        /* VCAM_I2C */
#ifdef NEED_MANUAL_VCAMI2C_POWER
		if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMI2C, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_I2C), power id = %d \n", VCAMI2C);
			goto _kdCISModulePowerOn_exit_;
		}
#endif

		if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV9734_MIPI_RAW))) {
			PK_ERR("[PowerDown]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx,currSensorName);

			/* Set XSHUTDN(RESET) Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, 0);
			mdelay(1);

			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAMD), power id = %d \n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			ISP_MCLK1_EN(0);
		} else {
			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
				PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n", VCAMIO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
		}
		/* GATE the cam_gate when sensor is not in use */
		set_gate_state(GATED);
	}
	PK_DBG(" --- SensorName: %s is going to power on, END --------- \n",currSensorName);
	return 0;

	_kdCISModulePowerOn_exit_:
	return -EIO;

}
EXPORT_SYMBOL(kdCISModulePowerOn);

static void gating_work_func(struct work_struct *work)
{
	mutex_lock(&cam_gate->mutex);
	if (cam_gate->req_state != cam_gate->cur_state) {
		mtkcam_gpio_set(0, CAMPDN, !cam_gate->req_state);
		conf_cam_gate(cam_gate->req_state);
		pr_info("%s: forcefully gating the cam sensor\n", __func__);
	}
	mutex_unlock(&cam_gate->mutex);
}

/* Can be tunned according to the expected UX reponse time of MUTE button */
#define WAIT_FOR_USERSPACE 999

static int cam_gating_callback(struct notifier_block *nb, unsigned long event,
	void *v)
{
	int update_state = 0;

	cancel_delayed_work_sync(&cam_gate->work);
	mutex_lock(&cam_gate->mutex);

	if (event == (unsigned long)UNGATED) {
		/* Update the req_state to allow the userspace to enable the
		 * the CAM (from kdCISModulePowerOn()) whenever its required
		 */
		cam_gate->req_state = UNGATED;
		PK_DBG("%s: enabling CAM, event=%lu \n", __func__, event);
	} else if(event == (unsigned long)GATED) {
		cam_gate->req_state = GATED;
		/* Make sure CAM gets disabled inorder to enable GATING mode,
		 * incase the userspace fails to close and disable cam in time
		 */
		if (cam_gate->cur_state != GATED)
			update_state = 1;
		PK_DBG("%s: disabling CAM, event=%lu \n", __func__, event);
	} else {
		PK_ERR("%s has nothing to do for event:%lu", __func__, event);
		mutex_unlock(&cam_gate->mutex);
		return NOTIFY_DONE;
	}

	mutex_unlock(&cam_gate->mutex);

	/* Wait for the usespace applications to close before GATING the CAM */
	if (update_state)
		schedule_delayed_work(&cam_gate->work,
					msecs_to_jiffies(WAIT_FOR_USERSPACE));

	return NOTIFY_OK;
}

static struct notifier_block cam_notifier = {
	.notifier_call = cam_gating_callback,
	.priority = 0,
};

#define AMZ_CAM_SENSOR_OF_NODE "camera1"

static int __init kd_camera_init(void)
{
	struct device_node *node = NULL;
	int gpio_pwr_en, ret;
	static struct cam_gating *gate;

	node = of_find_node_by_name(NULL, AMZ_CAM_SENSOR_OF_NODE);
	if (NULL == node) {
		PK_ERR("%s: could not find device tree node\n",
			__func__);
		ret = -ENODEV;
		goto err;
	}

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate) {
		ret = -ENOMEM;
		goto err;
	}

	gpio_pwr_en = of_get_named_gpio(node, "cam-pwr-en-gpios", 0);
	if (gpio_is_valid(gpio_pwr_en)) {
		gate->cam_pwr_en = gpio_to_desc(gpio_pwr_en);
	} else {
		ret = -EINVAL;
		PK_ERR("%s: failed to find cam-pwr-en\n", __func__);
		goto err;
	}

	ret = gpio_request(gpio_pwr_en, "cam_pwr_gate_pin");
	if (ret) {
		PK_ERR("%s:%u gpio req failed ret=%d\n",
			__func__, __LINE__, ret);
		goto err;
	}

	gpiod_direction_output(gate->cam_pwr_en , 1);
	INIT_DELAYED_WORK(&gate->work, gating_work_func);
	mutex_init(&gate->mutex);

	cam_gate = gate;

	ret = register_gating_state_notifier(&cam_notifier);
	if (ret != 0) {
		PK_ERR("%s: register gating mode callback fail\n",__func__);
		gpio_free(gpio_pwr_en);
		mutex_destroy(&gate->mutex);
		goto err;
	}

	return ret;
err:
	kfree(gate);
	return ret;
}

device_initcall_sync(kd_camera_init);
