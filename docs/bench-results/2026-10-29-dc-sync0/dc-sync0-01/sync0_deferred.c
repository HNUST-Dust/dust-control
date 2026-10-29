/* SPDX-License-Identifier: Apache-2.0 */
#include "sync0_deferred.h"
#if defined(CONFIG_DUST_ECAT_DC_SYNC)
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include "pdi_deferred.h"
static atomic_t pending;
static uint32_t first_at;
struct { uint32_t version, notifications, services, coalesced, fault,
                  period_cycles, max_lateness, max_work; }
    dust_sync0_stats = {1, 0, 0, 0, 0, 0, 0, 0};
void dust_sync0_reset(void) {
    atomic_set(&pending, 0);
    first_at = 0;
    dust_sync0_stats.fault = 0;
    dust_sync0_stats.period_cycles = 0;
}
void dust_sync0_period(uint32_t cycles) {
    dust_sync0_stats.fault = 0;
    dust_sync0_stats.period_cycles = cycles;
}
void dust_sync0_begin(void) {
    // Ignore clock-settling transients before the business session is armed.
    // SSC's own DC/SM watchdog remains enabled throughout startup.
    unsigned key = irq_lock();
    dust_sync0_stats.fault = 0;
    dust_sync0_stats.max_lateness = 0;
    dust_sync0_stats.max_work = 0;
    irq_unlock(key);
}
void dust_sync0_notify(void) {
    ++dust_sync0_stats.notifications;
    if (atomic_inc(&pending) == 0) first_at = k_cycle_get_32();
    else ++dust_sync0_stats.coalesced;
    dust_pdi_wake();
}
bool dust_sync0_error(void) { return dust_sync0_stats.fault != 0; }
bool dust_sync0_service(void (*handler)(void)) {
    // Atomically pair first timestamp with pending count. A new IRQ after
    // release owns a new timestamp, never overwriting this local snapshot.
    unsigned key = irq_lock();
    int count = atomic_set(&pending, 0);
    uint32_t at = first_at;
    irq_unlock(key);
    if (!count) return false;
    const uint32_t start = k_cycle_get_32();
    const uint32_t late = start - at;
    if (late > dust_sync0_stats.max_lateness) dust_sync0_stats.max_lateness = late;
    if (count > 1 && dust_sync0_stats.period_cycles) dust_sync0_stats.fault |= 1;
    if (dust_sync0_stats.period_cycles && late >= dust_sync0_stats.period_cycles)
        dust_sync0_stats.fault |= 2;
    ++dust_sync0_stats.services;
    // Invoke once, never replay a queue of stale business cycles. The handler
    // sees sticky faults before processing and publishes a protected image.
    handler();
    uint32_t work = k_cycle_get_32() - start;
    if (work > dust_sync0_stats.max_work) dust_sync0_stats.max_work = work;
    if (dust_sync0_stats.period_cycles && work >= dust_sync0_stats.period_cycles)
        dust_sync0_stats.fault |= 4;
    return true;
}
#endif
