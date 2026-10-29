/* SPDX-License-Identifier: Apache-2.0 */
#include "sync0_deferred.h"
#if defined(CONFIG_DUST_ECAT_DC_SYNC)
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include "pdi_deferred.h"
static atomic_t pending;
static uint32_t first_at;
static bool business_active;
static bool measuring;
static unsigned current_phase;
#define STAGES 11
static uint32_t phase_at, phase_cycles[STAGES], phase_worker[STAGES];
static volatile uint32_t worker_cycles;
static uint32_t phase_worker_at;
void dust_sync0_worker_elapsed(uint32_t elapsed) { worker_cycles += elapsed; }
struct {
    uint32_t version, services, stages, max_work, worst_lateness, reserved;
    uint64_t total[STAGES], worker_total[STAGES];
    uint32_t maximum[STAGES], worst[STAGES], worker_worst[STAGES], worker_at_max[STAGES];
} dust_sync0_phase_stats = {2, 0, STAGES, 0, 0, 0, {0}, {0}, {0}, {0}, {0}, {0}};
unsigned dust_sync0_phase(unsigned phase) {
    if (!measuring || phase >= STAGES) return 0;
    unsigned previous = current_phase;
    unsigned key = irq_lock();
    uint32_t now = k_cycle_get_32();
    uint32_t worker = worker_cycles;
    irq_unlock(key);
    phase_cycles[previous] += now - phase_at;
    phase_worker[previous] += worker - phase_worker_at;
    phase_worker_at = worker;
    phase_at = now;
    current_phase = phase;
    return previous;
}
struct { uint32_t version, services, coalesced, fault, max_lateness, max_work; }
    dust_sync0_active_stats = {1, 0, 0, 0, 0, 0};
void dust_sync0_set_active(bool active) { business_active = active; }
struct { uint32_t version, notifications, services, coalesced, fault,
                  period_cycles, max_lateness, max_work; }
    dust_sync0_stats = {1, 0, 0, 0, 0, 0, 0, 0};
void dust_sync0_reset(void) {
    atomic_set(&pending, 0);
    first_at = 0;
    business_active = false;
    measuring = false;
    dust_sync0_phase_stats.services = 0;
    dust_sync0_phase_stats.max_work = 0;
    dust_sync0_phase_stats.worst_lateness = 0;
    for (unsigned i = 0; i < STAGES; ++i) {
        dust_sync0_phase_stats.total[i] = 0;
        dust_sync0_phase_stats.worker_total[i] = 0;
        dust_sync0_phase_stats.worker_worst[i] = 0;
        dust_sync0_phase_stats.worker_at_max[i] = 0;
        dust_sync0_phase_stats.maximum[i] = 0;
        dust_sync0_phase_stats.worst[i] = 0;
    }
    dust_sync0_active_stats.services = 0;
    dust_sync0_active_stats.coalesced = 0;
    dust_sync0_active_stats.fault = 0;
    dust_sync0_active_stats.max_lateness = 0;
    dust_sync0_active_stats.max_work = 0;
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
    const bool active = business_active;
    key = irq_lock();
    const uint32_t start = k_cycle_get_32();
    phase_worker_at = worker_cycles;
    irq_unlock(key);
    const uint32_t late = start - at;
    if (late > dust_sync0_stats.max_lateness) dust_sync0_stats.max_lateness = late;
    if (count > 1 && dust_sync0_stats.period_cycles) dust_sync0_stats.fault |= 1;
    if (dust_sync0_stats.period_cycles && late >= dust_sync0_stats.period_cycles)
        dust_sync0_stats.fault |= 2;
    ++dust_sync0_stats.services;
    // Invoke once, never replay a queue of stale business cycles. The handler
    // sees sticky faults before processing and publishes a protected image.
    for (unsigned i = 0; i < STAGES; ++i) { phase_cycles[i] = 0; phase_worker[i] = 0; }
    current_phase = 0;
    phase_at = start;
    measuring = active;
    handler();
    key = irq_lock();
    uint32_t end = k_cycle_get_32();
    uint32_t worker_end = worker_cycles;
    irq_unlock(key);
    if (measuring) {
        phase_cycles[current_phase] += end - phase_at;
        phase_worker[current_phase] += worker_end - phase_worker_at;
    }
    measuring = false;
    uint32_t work = end - start;
    if (work > dust_sync0_stats.max_work) dust_sync0_stats.max_work = work;
    if (dust_sync0_stats.period_cycles && work >= dust_sync0_stats.period_cycles)
        dust_sync0_stats.fault |= 4;
    // Count only cycles Active at entry and exit; freeze through stop handling.
    if (active && business_active) {
        ++dust_sync0_phase_stats.services;
        for (unsigned i = 0; i < STAGES; ++i) {
            dust_sync0_phase_stats.total[i] += phase_cycles[i];
            dust_sync0_phase_stats.worker_total[i] += phase_worker[i];
            if (phase_cycles[i] > dust_sync0_phase_stats.maximum[i]) {
                dust_sync0_phase_stats.maximum[i] = phase_cycles[i];
                dust_sync0_phase_stats.worker_at_max[i] = phase_worker[i];
            }
        }
        if (work > dust_sync0_phase_stats.max_work) {
            dust_sync0_phase_stats.max_work = work;
            dust_sync0_phase_stats.worst_lateness = late;
            for (unsigned i = 0; i < STAGES; ++i) {
                dust_sync0_phase_stats.worst[i] = phase_cycles[i];
                dust_sync0_phase_stats.worker_worst[i] = phase_worker[i];
            }
        }
        ++dust_sync0_active_stats.services;
        if (count > 1) dust_sync0_active_stats.coalesced += count - 1;
        dust_sync0_active_stats.fault |= dust_sync0_stats.fault;
        if (late > dust_sync0_active_stats.max_lateness)
            dust_sync0_active_stats.max_lateness = late;
        if (work > dust_sync0_active_stats.max_work)
            dust_sync0_active_stats.max_work = work;
    }
    return true;
}
#endif
