/**
 * Author: Pere Tuset (peretuset@uoc.edu)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Alda Xhafa (axhafa@uoc.edu)
 * Date:   December 2016
 * Description: 
 */

#include "stdio.h"
#include "string.h"
#include "radiotimer.h"
#include "debugpins.h"

//=========================== variables =======================================

typedef struct {
   radiotimer_compare_cbt    overflow_cb;
   radiotimer_compare_cbt    compare_cb;
} radiotimer_vars_t;

radiotimer_vars_t radiotimer_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

//===== admin

void radiotimer_init() {
   // clear local variables
   memset(&radiotimer_vars,0,sizeof(radiotimer_vars_t));
}

void radiotimer_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.overflow_cb = cb;
}

void radiotimer_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.compare_cb = cb;
}

void radiotimer_setStartFrameCb(radiotimer_capture_cbt cb) {
   while(1);
}

void radiotimer_setEndFrameCb(radiotimer_capture_cbt cb) {
   while(1);
}

void radiotimer_start(PORT_RADIOTIMER_WIDTH period) {
}

//===== direct access

PORT_RADIOTIMER_WIDTH radiotimer_getValue() {
	return 0;
}

void radiotimer_setPeriod(PORT_RADIOTIMER_WIDTH period) {
}

PORT_RADIOTIMER_WIDTH radiotimer_getPeriod() {
	return 0;
}

//===== compare

void radiotimer_schedule(PORT_RADIOTIMER_WIDTH offset) {
}

void radiotimer_cancel() {
}

//===== capture

port_INLINE PORT_RADIOTIMER_WIDTH radiotimer_getCapturedTime() {
	return 0;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

kick_scheduler_t radiotimer_isr() {
   return DO_NOT_KICK_SCHEDULER;
}
