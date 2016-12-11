/**
 * Author: Pere Tuset (peretuset@uoc.edu)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Alda Xhafa (axhafa@uoc.edu)
 * Date:   December 2016
 * Description: 
 */

#include "string.h"
#include "bsp_timer.h"
#include "board.h"
#include "debugpins.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
	bsp_timer_cbt cb;
	PORT_TIMER_WIDTH last_compare_value;
	bool initiated;
	uint32_t tooclose;
	uint32_t diff;
} bsp_timer_vars_t;

bsp_timer_vars_t bsp_timer_vars;

//=========================== prototypes ======================================

void bsp_timer_isr_private(void);

//=========================== public ==========================================

/**
 \brief Initialize this module.

 This functions starts the timer, i.e. the counter increments, but doesn't set
 any compare registers, so no interrupt will fire.
 */
void bsp_timer_init() {
	// Clear local variables
	memset(&bsp_timer_vars, 0, sizeof(bsp_timer_vars_t));
}

/**
 \brief Register a callback.

 \param cb The function to be called when a compare event happens.
 */
void bsp_timer_set_callback(bsp_timer_cbt cb) {
	bsp_timer_vars.cb = cb;
}

/**
 \brief Reset the timer.

 This function does not stop the timer, it rather resets the value of the
 counter, and cancels a possible pending compare event.
 */
void bsp_timer_reset() {
	// reset timer
    bsp_timer_vars.initiated = 0;
	// record last timer compare value
	bsp_timer_vars.last_compare_value = 0;
}

/**
 \brief Schedule the callback to be called in some specified time.

 The delay is expressed relative to the last compare event. It doesn't matter
 how long it took to call this function after the last compare, the timer will
 expire precisely delayTicks after the last one.

 The only possible problem is that it took so long to call this function that
 the delay specified is shorter than the time already elapsed since the last
 compare. In that case, this function triggers the interrupt to fire right away.

 This means that the interrupt may fire a bit off, but this inaccuracy does not
 propagate to subsequent timers.

 \param delayTicks Number of ticks before the timer expired, relative to the
 last compare event.
 */
void bsp_timer_scheduleIn(PORT_TIMER_WIDTH delayTicks) {
}

/**
 \brief Cancel a running compare.
 */
void bsp_timer_cancel_schedule() {
}

/**
 \brief Return the current value of the timer's counter.

 \returns The current value of the timer's counter.
 */
PORT_TIMER_WIDTH bsp_timer_get_currentValue() {
	return 0;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

kick_scheduler_t bsp_timer_isr() {
	// call the callback
	bsp_timer_vars.cb();
	
	// kick the OS
	return KICK_SCHEDULER;
}

