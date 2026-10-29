/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#if defined(CONFIG_DUST_ECAT_DC_SYNC)
void dust_sync0_reset(void);
void dust_sync0_notify(void);
void dust_sync0_period(uint32_t cycles);
void dust_sync0_begin(void);
bool dust_sync0_service(void (*handler)(void));
bool dust_sync0_error(void);
#endif
#ifdef __cplusplus
}
#endif
