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

#include "prcm.h"
#include "vims.h"

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
	PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
  	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

	// Enable clock to GPIO module while CPU is running
  	PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
   	PRCMLoadSet();
  	while( ! PRCMLoadGet() );

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
