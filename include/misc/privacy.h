/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MISC_PRIVACY_H__
#define __MISC_PRIVACY_H__

#include <linux/types.h>

#if IS_ENABLED(CONFIG_GATING) || IS_ENABLED(CONFIG_KEYBOARD_GPIO_PRIVACY)
bool camera_shuttered(void);
#else
static inline bool camera_shuttered(void)
{
	return false;
}
#endif

#endif
