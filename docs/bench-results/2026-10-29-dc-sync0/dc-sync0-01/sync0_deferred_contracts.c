#include <assert.h>
#include <stdio.h>
#include "sync0_deferred.h"
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
    puts("Deferred Sync0 backlog/latency/work/wrap contracts PASS");
}
