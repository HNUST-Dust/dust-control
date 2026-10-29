#include <assert.h>
#include <stdio.h>
#include "sync0_deferred.h"
extern struct { uint32_t version, services, coalesced, fault, max_lateness, max_work; } dust_sync0_active_stats;
extern struct {
    uint32_t version, services, stages, max_work, worst_lateness, reserved;
    uint64_t total[8]; uint32_t maximum[8], worst[8];
} dust_sync0_phase_stats;
static uint32_t now;
static unsigned calls, wakes;
static bool inject, expect_fault;
uint32_t k_cycle_get_32(void) { return now; }
unsigned irq_lock(void) { return 0; }
void irq_unlock(unsigned key) { assert(!key); }
void dust_pdi_wake(void) { ++wakes; }
static void owner(void) {
    ++calls;
    assert(dust_sync0_error() == expect_fault);
    now += 5;
    if (inject) { inject=false; dust_sync0_notify(); }
}
static void stopping_owner(void) { now += 150; dust_sync0_set_active(false); }
static void phased_owner(void) {
    now += 2;
    unsigned previous = dust_sync0_phase(DUST_SYNC_PROCESS);
    now += 11;
    assert(dust_sync0_phase(DUST_SYNC_ESC) == DUST_SYNC_PROCESS);
    now += 17;
    dust_sync0_phase(previous);
    now += 3;
}
static void slow_owner(void) { ++calls; now += 100; }
int main(void) {
    dust_sync0_reset(); dust_sync0_period(100);
    now=10; dust_sync0_notify(); assert(calls==0 && wakes==1);
    now=20; assert(dust_sync0_service(owner)); assert(!dust_sync0_error());
    assert(!dust_sync0_service(owner));
    now=30; dust_sync0_notify(); inject=true;
    assert(dust_sync0_service(owner)); assert(dust_sync0_service(owner));
    // Backlog is a sticky fault and handler sees it before business execution.
    dust_sync0_notify(); dust_sync0_notify(); expect_fault=true;
    assert(dust_sync0_service(owner)); assert(dust_sync0_error());
    assert(!dust_sync0_service(owner));
    // Unsigned timer wrap is valid for bounded per-period intervals.
    dust_sync0_reset(); dust_sync0_period(100); expect_fault=false;
    now=0xfffffff0U; dust_sync0_notify(); now=5;
    assert(dust_sync0_service(owner)); assert(!dust_sync0_error());
    dust_sync0_notify(); now += 100; expect_fault=true;
    assert(dust_sync0_service(owner)); assert(dust_sync0_error());
    dust_sync0_reset(); dust_sync0_period(100);
    dust_sync0_notify(); assert(dust_sync0_service(slow_owner));
    assert(dust_sync0_error());
    dust_sync0_reset(); dust_sync0_period(100); expect_fault=false;
    dust_sync0_notify(); assert(dust_sync0_service(owner));
    assert(dust_sync0_active_stats.services == 0);
    dust_sync0_set_active(true);
    dust_sync0_notify(); now += 7; assert(dust_sync0_service(owner));
    assert(dust_sync0_active_stats.services == 1);
    assert(dust_sync0_active_stats.max_lateness == 7);
    assert(dust_sync0_active_stats.max_work == 5);
    dust_sync0_notify(); assert(dust_sync0_service(stopping_owner));
    assert(dust_sync0_active_stats.services == 1);
    assert(dust_sync0_active_stats.max_work == 5);
    dust_sync0_reset(); dust_sync0_period(100);
    dust_sync0_set_active(true);
    now=100; dust_sync0_notify(); now += 7;
    assert(dust_sync0_service(phased_owner));
    assert(dust_sync0_phase_stats.services == 1);
    assert(dust_sync0_phase_stats.max_work == 33);
    assert(dust_sync0_phase_stats.worst_lateness == 7);
    assert(dust_sync0_phase_stats.total[DUST_SYNC_SSC] == 5);
    assert(dust_sync0_phase_stats.total[DUST_SYNC_PROCESS] == 11);
    assert(dust_sync0_phase_stats.total[DUST_SYNC_ESC] == 17);
    uint32_t sum = 0;
    for (unsigned i=0; i<8; ++i) sum += dust_sync0_phase_stats.worst[i];
    assert(sum == 33);
    dust_sync0_notify(); assert(dust_sync0_service(stopping_owner));
    assert(dust_sync0_phase_stats.services == 1);
    assert(dust_sync0_phase(DUST_SYNC_CAN) == 0);
    puts("Deferred Sync0 backlog/latency/work/wrap contracts PASS");
}
