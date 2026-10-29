/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef HPM5E31_PHY_H_
#define HPM5E31_PHY_H_

#include <zephyr/device.h>
#include "hpm_common.h"

void ecat_phy_reset(const struct device *esc_dev);
void ecat_phy_keep_reset(const struct device *esc_dev);
hpm_stat_t ecat_phy_config(const struct device *esc_dev);
void ecat_phy_update_link(const struct device *esc_dev);

#if defined(CONFIG_DUST_ECAT_DC_SYNC)
/* Owner-only incremental link poll; true means this poll is complete. */
bool ecat_phy_update_link_step(const struct device *esc_dev);
#endif

#endif
