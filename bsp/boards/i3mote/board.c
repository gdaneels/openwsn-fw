
/**
 * Author: Pere Tuset (peretuset@uoc.edu)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Alda Xhafa (axhafa@uoc.edu)
 * Date:   December 2016
 * Description: 
 */

#include "board.h"
#include "leds.h"
#include "bsp_timer.h"
#include "radiotimer.h"
#include "debugpins.h"
#include "uart.h"
#include "radio.h"
#include "i2c.h"
#include "sensors.h"

#include "ti-lib.h"
#include "oscillators.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

extern int mote_main(void);

int main(void) {
   return mote_main();
}

//=========================== public ==========================================

void delay(void) {
   volatile uint16_t delay;
   for (delay=0xffff;delay>0;delay--);
}

void board_init(void) {

    /* Enable flash cache and prefetch. */
    ti_lib_vims_mode_set(VIMS_BASE, VIMS_MODE_ENABLED);
    ti_lib_vims_configure(VIMS_BASE, true, true);

    ti_lib_int_master_disable();

    /* Set the LF XOSC as the LF system clock source */
    oscillators_select_lf_xosc();

    //lpm_init();

    board_init();

	/* Turn on the PERIPH PD */
    ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
    while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH)
         != PRCM_DOMAIN_POWER_ON));

	/* Enable GPIO peripheral */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

    /* Apply settings and wait for them to take effect */
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

	leds_init();

  	while(true) {
  		leds_error_toggle();
  		leds_sync_toggle();
  		delay();
  	}
}

/**
 * Puts the board to sleep
 */
void board_sleep(void) {
}

/**
 * Timer runs at 32 MHz and is 32-bit wide
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
void board_timer_init(void) {
}

/**
 * Returns the current value of the timer
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
uint32_t board_timer_get(void) {
  return 0;
}

/**
 * Returns true if the timer has expired
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
bool board_timer_expired(uint32_t future) {
  return 0;
}

/**
 * Resets the board
 */
void board_reset(void) {
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
