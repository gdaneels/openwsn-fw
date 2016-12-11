/**
 * Author: Pere Tuset (peretuset@uoc.edu)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Alda Xhafa (axhafa@uoc.edu)
 * Date:   December 2016
 * Description: 
 */

#include "stdint.h"
#include "leds.h"
#include "board.h"

#include "ioc.h"
#include "gpio.h"

//=========================== defines =========================================

#define RED_LED       ( 5 )
#define GREEN_LED     ( 6 )

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void leds_init(void) {
	IOCPinTypeGpioOutput(IOID_5);
	IOCPinTypeGpioOutput(IOID_6);

	GPIO_setOutputEnableDio(GREEN_LED, GPIO_OUTPUT_ENABLE);
	GPIO_setOutputEnableDio(RED_LED, GPIO_OUTPUT_ENABLE);
}

void leds_error_on(void) {
	GPIO_setDio(RED_LED);
}

void leds_error_off(void) {
	GPIO_clearDio(RED_LED);
}

void leds_error_toggle(void) {
	GPIO_toggleDio(RED_LED);
}

uint8_t leds_error_isOn(void) {
	return GPIO_readDio(RED_LED) & 0xFF;
}

void leds_sync_on(void) {
	GPIO_setDio(GREEN_LED);
}

void leds_sync_off(void) {
	GPIO_clearDio(GREEN_LED);
}

void leds_sync_toggle(void) {
	GPIO_toggleDio(GREEN_LED);
}

uint8_t leds_sync_isOn(void) {
	return GPIO_readDio(GREEN_LED) & 0xFF;
}

void leds_radio_on(void) {
}

void leds_radio_off(void) {
}

void leds_radio_toggle(void) {
}

uint8_t leds_radio_isOn(void) {
	return 0;
}

void leds_debug_on(void) {
}

void leds_debug_off(void) {
}

void leds_debug_toggle(void) {
}

uint8_t leds_debug_isOn(void) {
	return 0;
}

void leds_all_on(void) {
}

void leds_all_off(void) {
}

void leds_all_toggle(void) {
}

void leds_error_blink(void) {
}

void leds_circular_shift(void) {
}

void leds_increment(void) {
}

//=========================== private =========================================
