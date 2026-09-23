/*
 * Copyright 2018 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "gpio-privacy: " fmt

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/input.h>
#include <misc/privacy.h>

#define DEFAULT_DEBOUNCE_INTERVAL 5

enum privacy_state {
	PRIVACY_STATE_OFF = 0,  /* HW privacy is OFF */
	PRIVACY_STATE_ON,       /* HW privacy is ON */
};

struct privacy_state_warning_event {
	const char *desc;
	unsigned int input_type;
	unsigned int code;
	unsigned int input_value;
	struct input_dev *input_dev;
	struct work_struct work;
	struct privacy_priv *priv;
};

struct privacy_state_event {
	const char *desc;
	unsigned int input_type;
	unsigned int code;
	struct input_dev *input_dev;
	struct work_struct work;
	int state_gpio;
	enum of_gpio_flags state_gpio_flags;
	struct privacy_priv *priv;
};

struct privacy_button_event {
	const char *desc;
	unsigned int code;
	unsigned int input_type;
	int debounce_interval;
	struct input_dev *input_dev;
	struct delayed_work work;
	bool wakeup_capable;
	int button_gpio;
	enum of_gpio_flags button_gpio_flags;
	int last_button_event;
	unsigned long last_button_press_time;
	unsigned long last_button_release_time;
	struct privacy_priv *priv;
};

struct privacy_priv {
	int enable_gpio;
	enum of_gpio_flags enable_gpio_flags;
	int enable_gpio_toggle_duration;
	int privacy_event_max_press_duration;
	int auto_toggle_enable_gpio_time;
	bool is_desired_privacy_state_on;
	struct mutex mutex;
	struct privacy_state_warning_event *state_warning_event;
	struct privacy_state_event *state_event;
	struct privacy_button_event *button_event;
	struct delayed_work work;
};

static struct privacy_priv *privacy;

/* Forward declarations: */
static enum privacy_state __privacy_state(struct privacy_priv *priv);
static int __set_privacy_enable(struct privacy_priv *priv);

static void handle_privacy_button_event(struct privacy_button_event *button_event,
						bool button_pressed)
{
	enum privacy_state cur_state;
	struct privacy_priv *priv = button_event->priv;
	struct privacy_state_warning_event *state_warning_event = priv->state_warning_event;
	unsigned long button_press_duration;
	unsigned long last_button_press_time;
	bool is_desired_privacy_state_on;

	cur_state = __privacy_state(priv);

	if (button_pressed) {
		mutex_lock(&priv->mutex);
		button_event->last_button_press_time = jiffies;

		/* With the Silego chip, we only know whether we are entering
		 * privacy state on the button press because on button press
		 * Silego locks the current state, so save this off because
		 * we will need this info on button release.
		 */
		if (cur_state == PRIVACY_STATE_OFF)
			priv->is_desired_privacy_state_on = true;

		mutex_unlock(&priv->mutex);

		pr_debug("%s: privacy button PRESSED cur_state=%d\n", __func__, cur_state);
		return;
	}
	pr_debug("%s: privacy button RELEASED cur_state=%d\n", __func__, cur_state);

	/* privacy button released ! */

	mutex_lock(&priv->mutex);
	button_event->last_button_release_time = jiffies;
	last_button_press_time = button_event->last_button_press_time;
	is_desired_privacy_state_on = priv->is_desired_privacy_state_on;
	priv->is_desired_privacy_state_on = false;
	mutex_unlock(&priv->mutex);

	if (priv->privacy_event_max_press_duration > 0) {
		button_press_duration = last_button_press_time +
			msecs_to_jiffies(priv->privacy_event_max_press_duration);

		/* Ignore long press because it might be for Power Event or Factory Reset */
		if (time_after(jiffies, button_press_duration)) {
			pr_debug("%s: POWER EVENT: cur_state=%d\n", __func__, cur_state);
			is_desired_privacy_state_on = false;
		}
	}

	if (is_desired_privacy_state_on) {
		if (state_warning_event != NULL) {
			pr_debug("%s: state_warning schedule work: cur_state=%d\n", __func__, cur_state);
			schedule_work(&state_warning_event->work);
		}

		if (priv->auto_toggle_enable_gpio_time >= 0) {
			pr_info("%s: auto-toggle-enable schedule work\n", __func__);
			schedule_delayed_work(&priv->work,
				msecs_to_jiffies(priv->auto_toggle_enable_gpio_time));
		}
	}
}

static void privacy_work_func(struct work_struct *work)
{
	struct privacy_priv *priv =
		container_of(work, struct privacy_priv, work.work);

	mutex_lock(&priv->mutex);
	pr_info("%s: privacy state enable immediately\n",  __func__);
	__set_privacy_enable(priv);
	mutex_unlock(&priv->mutex);
}

static void privacy_state_warning_event_work_func(struct work_struct *work)
{
	struct privacy_state_warning_event *state_warning_event =
		container_of(work, struct privacy_state_warning_event, work);

	pr_info("%s: sending state warning to userspace. input_type=0x%x code=0x%x\n",
		__func__, state_warning_event->input_type, state_warning_event->code);

	if (state_warning_event->input_type == EV_KEY || state_warning_event->input_type == EV_SW) {

		/*
		 * EV_KEY key events and EV_SW switch events require a full transition from 0 to 1
		 * and then 1 to 0 in order to get future events
		 */
		input_event(state_warning_event->input_dev, state_warning_event->input_type,
				state_warning_event->code, 1);
		input_sync(state_warning_event->input_dev);

		input_event(state_warning_event->input_dev, state_warning_event->input_type,
				state_warning_event->code, 0);
		input_sync(state_warning_event->input_dev);

	} else if (state_warning_event->input_type == EV_MSC || state_warning_event->code == MSC_RAW) {

		/*
		 * EV_MSC events only send a single event with 'input value' from dts
		 */
		input_event(state_warning_event->input_dev, state_warning_event->input_type,
				state_warning_event->code, state_warning_event->input_value);
		input_sync(state_warning_event->input_dev);
	}
}

static void privacy_state_event_work_func(struct work_struct *work)
{
	bool value;
	struct privacy_state_event *state_event =
		container_of(work, struct privacy_state_event, work);

	value = gpio_get_value_cansleep(state_event->state_gpio);

	if (state_event->state_gpio_flags & OF_GPIO_ACTIVE_LOW)
		value = !value;

	input_event(state_event->input_dev, state_event->input_type,
			state_event->code, value);
	input_sync(state_event->input_dev);
}

static void privacy_button_event_work_func(struct work_struct *work)
{
	int value;
	struct privacy_button_event *button_event =
		container_of(work, struct privacy_button_event, work.work);

	value = gpio_get_value_cansleep(button_event->button_gpio);
	if (unlikely(value < 0)) {
		/*
		 * gpio read can fail, however we should report button
		 * press in order to notify userspace that privacy
		 * state has been changed.  force it to
		 * !button_event->last_button_event for that case in the hope
		 * we just missed one press or release.
		 */
		pr_warn_ratelimited("gpio-privacy: gpio %d read failed=%d\n",
				    button_event->button_gpio, value);
		value = !button_event->last_button_event;
	} else if (button_event->button_gpio_flags & OF_GPIO_ACTIVE_LOW) {
		value = !value;
	}

	if (button_event->last_button_event == value) {
		/*
		 * We can reach here when :
		 * 1) previous press/release has been canceled due to
		 *    debouce interval.
		 * 2) gpio_get_value() failed.
		 *
		 * We should report button press by all means in order for
		 * userspace to be notified about new privacy mode change.
		 * Thus send out an artificial event.
		 */
		handle_privacy_button_event(button_event, !value);
		input_event(button_event->input_dev, button_event->input_type,
				button_event->code, !value);
		input_sync(button_event->input_dev);
	} else {
		button_event->last_button_event = value;
	}

	handle_privacy_button_event(button_event, value);
	input_event(button_event->input_dev, button_event->input_type,
			button_event->code, value);
	input_sync(button_event->input_dev);

	if (button_event->wakeup_capable)
		pm_relax(button_event->input_dev->dev.parent);
}

static irqreturn_t privacy_state_interrupt(int irq, void *arg)
{
	struct privacy_state_event *state_event = arg;
	schedule_work(&state_event->work);
	return IRQ_HANDLED;
}

static irqreturn_t privacy_button_interrupt(int irq, void *arg)
{
	struct privacy_button_event *button_event = arg;

	if (button_event->wakeup_capable)
		pm_stay_awake(button_event->input_dev->dev.parent);

	cancel_delayed_work(&button_event->work);
	schedule_delayed_work(&button_event->work,
			msecs_to_jiffies(button_event->debounce_interval));

	return IRQ_HANDLED;
}

static int privacy_request_interrupts(struct platform_device *pdev)
{
	int ret;
	struct privacy_priv *priv = platform_get_drvdata(pdev);
	struct privacy_state_event *state_event = priv->state_event;
	struct privacy_button_event *button_event = priv->button_event;

	ret = devm_request_irq(&pdev->dev,
			gpio_to_irq(state_event->state_gpio),
			privacy_state_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"gpio-privacy-state", state_event);
	if (ret)
		return ret;

	ret = devm_request_irq(&pdev->dev,
			gpio_to_irq(button_event->button_gpio),
			privacy_button_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"gpio-privacy", button_event);
	if (ret)
		return ret;

	return 0;
}

static int privacy_setup_state_warning_event(struct platform_device *pdev)
{
	int ret;
	struct input_dev *input;
	struct device *dev = &pdev->dev;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	/* state_warning_event is optional dts node */
	if (priv->state_warning_event == NULL)
		return 0;

	INIT_WORK(&priv->state_warning_event->work,
				privacy_state_warning_event_work_func);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "gpio-privacy-state-warning";
	input->dev.parent = &pdev->dev;

	input_set_capability(input, priv->state_warning_event->input_type,
				priv->state_warning_event->code);

	priv->state_warning_event->input_dev = input;
	priv->state_warning_event->priv = priv;

	ret = input_register_device(input);
	if (ret)
		return ret;

	return 0;
}

static int privacy_setup_state_event(struct platform_device *pdev)
{
	int ret;
	struct input_dev *input;
	struct device *dev = &pdev->dev;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	INIT_WORK(&priv->state_event->work,
				privacy_state_event_work_func);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "gpio-privacy-state";
	input->dev.parent = &pdev->dev;

	input_set_capability(input, priv->state_event->input_type,
				priv->state_event->code);

	priv->state_event->input_dev = input;
	priv->state_event->priv = priv;

	ret = input_register_device(input);
	if (ret)
		return ret;

	/* seed initial value if already in a muted state */
	if (priv->state_event->input_type == EV_SW && __privacy_state(priv)) {
		input_event(priv->state_event->input_dev,
				priv->state_event->input_type,
				priv->state_event->code, 1);
		input_sync(priv->state_event->input_dev);
	}

	return 0;
}

static int privacy_setup_button_event(struct platform_device *pdev)
{
	int ret;
	struct input_dev *input;
	struct device *dev = &pdev->dev;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	INIT_DELAYED_WORK(&priv->button_event->work,
				privacy_button_event_work_func);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "gpio-privacy-button";
	input->dev.parent = &pdev->dev;

	input_set_capability(input, priv->button_event->input_type,
				priv->button_event->code);

	priv->button_event->input_dev = input;
	priv->button_event->priv = priv;

	ret = input_register_device(input);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_OF
static int privacy_state_warning_event_parse_of(struct platform_device *pdev)
{
	struct device_node *node;
	struct device_node *state_warning_event_node;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	node = pdev->dev.of_node;

	state_warning_event_node = of_get_child_by_name(node, "state_warning_event");
	if (!state_warning_event_node) {
		/* state warning event is optional in dts */
		dev_warn(&pdev->dev, "No state warning event configured in dts\n");
		return 0;
	}

	priv->state_warning_event = devm_kzalloc(&pdev->dev, sizeof(*priv->state_warning_event),
					 GFP_KERNEL);
	if (!priv->state_warning_event)
		return -ENOMEM;

	priv->state_warning_event->desc = of_get_property(state_warning_event_node, "label",
						  NULL);

	if (of_property_read_u32(state_warning_event_node, "linux,input-type",
				 &priv->state_warning_event->input_type))
		priv->state_warning_event->input_type = EV_KEY;

	if (of_property_read_u32(state_warning_event_node, "linux,code",
				 &priv->state_warning_event->code))
		return -EINVAL;

	if (priv->state_warning_event->input_type == EV_MSC && priv->state_warning_event->code == MSC_RAW) {
		if (of_property_read_u32(state_warning_event_node, "linux,input-value",
					&priv->state_warning_event->input_value))
			return -EINVAL;
	}
	return 0;
}

static int privacy_state_event_parse_of(struct platform_device *pdev)
{
	int ret;
	enum of_gpio_flags flags;
	struct device_node *node;
	struct device_node *state_event_node;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	node = pdev->dev.of_node;

	state_event_node = of_get_child_by_name(node, "state_event");
	if (!state_event_node) {
		dev_err(&pdev->dev, "No state event configured in dts\n");
		return -EINVAL;
	}

	priv->state_event = devm_kzalloc(&pdev->dev, sizeof(*priv->state_event),
					 GFP_KERNEL);
	if (!priv->state_event)
		return -ENOMEM;

	priv->state_event->desc = of_get_property(state_event_node, "label",
						  NULL);

	if (of_property_read_u32(state_event_node, "linux,input-type",
				 &priv->state_event->input_type))
		priv->state_event->input_type = EV_KEY;

	if (of_property_read_u32(state_event_node, "linux,code",
				 &priv->state_event->code))
		return -EINVAL;

	priv->state_event->state_gpio = of_get_gpio_flags(state_event_node, 0,
							&flags);

	if (!gpio_is_valid(priv->state_event->state_gpio)) {
		dev_err(&pdev->dev, "No state gpios configured in dts\n");
		return -EINVAL;
	}

	priv->state_event->state_gpio_flags = flags;

	ret = devm_gpio_request_one(&pdev->dev,
					priv->state_event->state_gpio,
					GPIOF_IN, "privacy-state-gpio");

	if (ret)
		return ret;

	dev_info(&pdev->dev, "state gpio %d configured.\n",
			priv->state_event->state_gpio);

	return 0;
}

static int privacy_button_event_parse_of(struct platform_device *pdev)
{
	int ret;
	enum of_gpio_flags flags;
	struct device_node *node;
	struct device_node *button_event_node;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	node = pdev->dev.of_node;

	button_event_node = of_get_child_by_name(node, "button_event");
	if (!button_event_node) {
		dev_err(&pdev->dev, "No button event configured in dts\n");
		return -EINVAL;
	}

	priv->button_event = devm_kzalloc(&pdev->dev, sizeof(*priv->button_event),
					 GFP_KERNEL);
	if (!priv->button_event)
		return -ENOMEM;

	priv->button_event->desc = of_get_property(button_event_node, "label",
						  NULL);

	if (of_property_read_u32(button_event_node, "linux,input-type",
				 &priv->button_event->input_type))
		priv->button_event->input_type = EV_KEY;

	if (of_property_read_u32(button_event_node, "linux,code",
				 &priv->button_event->code))
		return -EINVAL;

	if (of_property_read_u32(button_event_node, "debounce-interval",
				 &priv->button_event->debounce_interval))
		priv->button_event->debounce_interval =
			DEFAULT_DEBOUNCE_INTERVAL;

	priv->button_event->button_gpio = of_get_gpio_flags(button_event_node, 0,
							   &flags);

	if (!gpio_is_valid(priv->button_event->button_gpio)) {
		dev_err(&pdev->dev, "No button gpios configured in dts\n");
		return -EINVAL;
	}

	priv->button_event->button_gpio_flags = flags;

	ret = devm_gpio_request_one(&pdev->dev,
					priv->button_event->button_gpio,
					GPIOF_IN, "privacy-button-gpio");
	if (ret)
		return ret;

	dev_info(&pdev->dev, "button gpio %d configured.\n",
			priv->button_event->button_gpio);

	priv->button_event->wakeup_capable =
			of_property_read_bool(button_event_node, "wakeup-source");

	return 0;
}

static int privacy_parse_of(struct platform_device *pdev)
{
	enum of_gpio_flags flags;
	int gpio, ret, gpio_init_val;
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	gpio = of_get_named_gpio_flags(pdev->dev.of_node, "enable-gpio", 0,
				       &flags);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "No enable gpio configured in dts\n");
		return -EINVAL;
	}

	if (flags & OF_GPIO_ACTIVE_LOW)
		gpio_init_val = GPIOF_OUT_INIT_HIGH;
	else
		gpio_init_val = GPIOF_OUT_INIT_LOW;

	ret = devm_gpio_request_one(&pdev->dev, gpio,
					gpio_init_val,
					"privacy-enable-gpio");
	if (ret)
		return ret;

	priv->enable_gpio = gpio;
	priv->enable_gpio_flags = flags;
	priv->is_desired_privacy_state_on = false;

	if (of_property_read_u32(pdev->dev.of_node,
				"enable-gpio-toggle-duration",
				&priv->enable_gpio_toggle_duration))
		priv->enable_gpio_toggle_duration = 0;

	if (of_property_read_u32(pdev->dev.of_node,
				"auto-toggle-enable-gpio-time",
				 &priv->auto_toggle_enable_gpio_time))
		priv->auto_toggle_enable_gpio_time = -1;

	if (of_property_read_u32(pdev->dev.of_node,
				"privacy-event-max-press-duration",
				&priv->privacy_event_max_press_duration))
		priv->privacy_event_max_press_duration = 0;

	if (priv->auto_toggle_enable_gpio_time >= 0)
		INIT_DELAYED_WORK(&priv->work, privacy_work_func);

	return 0;
}
#else
static int privacy_button_event_parse_of(struct platform_device *pdev)
{
	return -EINVAL;
}

static int privacy_parse_of(struct platform_device *pdev)
{
	return -EINVAL;
}
#endif

static enum privacy_state __privacy_state(struct privacy_priv *priv)
{
	struct privacy_state_event *state_event = priv->state_event;

	int value = gpio_get_value_cansleep(state_event->state_gpio);

	if ((!value &&
		state_event->state_gpio_flags & OF_GPIO_ACTIVE_LOW) ||
		(value &&
		!(state_event->state_gpio_flags & OF_GPIO_ACTIVE_LOW)))
		/* return true when privacy state is on */
		return PRIVACY_STATE_ON;

	return PRIVACY_STATE_OFF;
}

bool camera_shuttered(void)
{
	if (!privacy)
		return false;

	return __privacy_state(privacy) == PRIVACY_STATE_ON;
}
EXPORT_SYMBOL(camera_shuttered);

static int __set_privacy_enable(struct privacy_priv *priv)
{
	int i = 0;
	int value = 1; /* default to 1, active high, unless proven otherwise */
	const int max_wait = 100;

	pr_info("%s: Enter\n",  __func__);

	if (priv->enable_gpio_flags & OF_GPIO_ACTIVE_LOW)
		value = 0;

	gpio_set_value(priv->enable_gpio, value);

	if (priv->enable_gpio_toggle_duration > 0) {
		/*
		 * toggle enable_gpio for specified duration but do not
		 * wait for privacy enabled
		 */
		if (priv->enable_gpio_toggle_duration < 20)
			usleep_range((priv->enable_gpio_toggle_duration * 1000),
				(priv->enable_gpio_toggle_duration * 1000) + 100);
		else
			msleep(priv->enable_gpio_toggle_duration);
	} else {
		/*
		 * wait for privacy enabled for up to 100ms or when
		 * privacy state is set (which ever comes first)
		 */
		while (i < max_wait) {
			if (__privacy_state(priv))
				break;
			usleep_range(1000, 1100);
			i++;
		}
	}

	gpio_set_value(priv->enable_gpio, !value);
	pr_info("%s: Leave\n",  __func__);

	if (i == max_wait)
		return -ETIMEDOUT;

	return 0;
}

static enum privacy_state privacy_state(struct device *dev)
{
	enum privacy_state cur_state;
	struct platform_device *pdev = to_platform_device(dev);
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	mutex_lock(&priv->mutex);
	cur_state = __privacy_state(priv);
	mutex_unlock(&priv->mutex);

	return cur_state;
}

static ssize_t show_privacy_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	enum privacy_state state = privacy_state(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", state);
}

static int set_privacy_enable(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct privacy_priv *priv = platform_get_drvdata(pdev);

	mutex_lock(&priv->mutex);
	pr_info("%s: privacy state enable immediately\n",  __func__);
	__set_privacy_enable(priv);
	mutex_unlock(&priv->mutex);

	return ret;
}

static ssize_t store_privacy_enable(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int enable, ret;

	if (!kstrtoint(buf, 10, &enable)) {

		/*
		 * Don't allow userspace to turn off Privacy Mode because
		 * privacy hardware circuit won't allow it.
		 */
		if (enable == PRIVACY_STATE_OFF)
			return -EINVAL;

		ret = set_privacy_enable(dev);
		if (ret)
			return ret;
	} else {
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IWGRP,
		   NULL, store_privacy_enable);
static DEVICE_ATTR(state, S_IRUGO, show_privacy_state, NULL);

static struct attribute *gpio_privacy_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_state.attr,
	NULL,
};

static struct attribute_group gpio_privacy_attr_group = {
	.attrs = gpio_privacy_attrs,
};

static int gpio_privacy_probe(struct platform_device *pdev)
{
	int ret;
	struct privacy_priv *priv;
	struct device *dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);

	platform_set_drvdata(pdev, priv);

	ret = privacy_parse_of(pdev);
	if (ret) {
		pr_err("failed to parse device tree = %d\n", ret);
		return ret;
	}

	ret = privacy_state_warning_event_parse_of(pdev);
	if (ret) {
		pr_err("failed to parse state warning event device tree = %d\n", ret);
		return ret;
	}

	ret = privacy_state_event_parse_of(pdev);
	if (ret) {
		pr_err("failed to parse state event device tree = %d\n", ret);
		return ret;
	}

	ret = privacy_button_event_parse_of(pdev);
	if (ret) {
		pr_err("failed to parse button event device tree = %d\n", ret);
		return ret;
	}

	ret = privacy_setup_state_warning_event(pdev);
	if (ret) {
		pr_err("failed to setup state warning event = %d\n", ret);
		return ret;
	}

	ret = privacy_setup_state_event(pdev);
	if (ret) {
		pr_err("failed to setup state event = %d\n", ret);
		return ret;
	}

	ret = privacy_setup_button_event(pdev);
	if (ret) {
		pr_err("failed to setup button event = %d\n", ret);
		return ret;
	}

	ret = privacy_request_interrupts(pdev);
	if (ret) {
		pr_err("failed to request interrupt = %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &gpio_privacy_attr_group);
	if (ret) {
		pr_err("failed to create sysfs group = %d\n", ret);
		return ret;
	}

	device_init_wakeup(&pdev->dev, priv->button_event->wakeup_capable);

	privacy = priv;

	return 0;
}

static int gpio_privacy_remove(struct platform_device *pdev)
{
	struct privacy_priv *priv;
	struct device *dev = &pdev->dev;
	struct privacy_state_warning_event *state_warning_event;
	struct privacy_state_event *state_event;
	struct privacy_button_event *button_event;

	priv = platform_get_drvdata(pdev);
	state_warning_event = priv->state_warning_event;
	state_event = priv->state_event;
	button_event = priv->button_event;

	privacy = NULL;

	if (priv->auto_toggle_enable_gpio_time >= 0)
		cancel_delayed_work_sync(&priv->work);
	if (state_warning_event != NULL)
		cancel_work_sync(&state_warning_event->work);
	cancel_work_sync(&state_event->work);
	cancel_delayed_work_sync(&button_event->work);

	sysfs_remove_group(&dev->kobj, &gpio_privacy_attr_group);

	if (state_warning_event != NULL)
		pm_relax(state_warning_event->input_dev->dev.parent);
	pm_relax(state_event->input_dev->dev.parent);
	pm_relax(button_event->input_dev->dev.parent);
	mutex_destroy(&priv->mutex);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_privacy_suspend(struct device *dev)
{
	struct privacy_priv *priv;
	struct privacy_state_event *state_event;
	struct privacy_button_event *button_event;
	struct platform_device *pdev = to_platform_device(dev);

	priv = platform_get_drvdata(pdev);
	state_event = priv->state_event;
	button_event = priv->button_event;

	if (button_event->wakeup_capable) {
		enable_irq_wake(gpio_to_irq(button_event->button_gpio));
	}

	return 0;
}

static int gpio_privacy_resume(struct device *dev)
{
	struct privacy_priv *priv;
	struct privacy_state_event *state_event;
	struct privacy_button_event *button_event;
	struct platform_device *pdev = to_platform_device(dev);

	priv = platform_get_drvdata(pdev);
	state_event = priv->state_event;
	button_event = priv->button_event;

	if (button_event->wakeup_capable) {
		disable_irq_wake(gpio_to_irq(button_event->button_gpio));
	}

	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id privacy_of_table[] = {
	{ .compatible = "gpio-privacy", },
	{ },
};
MODULE_DEVICE_TABLE(of, privacy_of_table);
#endif

#ifdef CONFIG_PM_SLEEP
static SIMPLE_DEV_PM_OPS(gpio_privacy_pm_ops, gpio_privacy_suspend, gpio_privacy_resume);
#endif

static struct platform_driver gpio_privacy_driver = {
	.driver = {
		.name = "gpio-privacy",
#ifdef CONFIG_PM_SLEEP
		.pm = &gpio_privacy_pm_ops,
#endif
		.of_match_table	= of_match_ptr(privacy_of_table),
	},
	.probe	= gpio_privacy_probe,
	.remove	= gpio_privacy_remove,
};

static int __init gpio_privacy_init(void)
{
	return platform_driver_register(&gpio_privacy_driver);
}

static void __exit gpio_privacy_exit(void)
{
	platform_driver_unregister(&gpio_privacy_driver);
}

module_init(gpio_privacy_init);
module_exit(gpio_privacy_exit);
MODULE_LICENSE("GPL");
