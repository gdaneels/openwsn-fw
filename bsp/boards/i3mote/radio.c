/**
 * Author: Pere Tuset (peretuset@uoc.edu)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Alda Xhafa (axhafa@uoc.edu)
 * Date:   December 2016
 * Description: 
 */

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "stdio.h"
#include "string.h"
#include "radiotimer.h"
#include "debugpins.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   radiotimer_capture_cbt    startFrame_cb;
   radiotimer_capture_cbt    endFrame_cb;
   radio_state_t             state; 
} radio_vars_t;

radio_vars_t radio_vars;

//=========================== prototypes ======================================

void     enable_radio_interrupts(void);
void     disable_radio_interrupts(void);

void     radio_on(void);
void     radio_off(void);

void     radio_error_isr(void);
void     radio_isr_internal(void);

//=========================== public ==========================================

//===== admin

void radio_init() {
   
   // clear variables
   memset(&radio_vars,0,sizeof(radio_vars_t));
   
   // change state
   radio_vars.state          = RADIOSTATE_STOPPED;
   
   // change state
   radio_vars.state          = RADIOSTATE_RFOFF;
}

void radio_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_setOverflowCb(cb);
}

void radio_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_setCompareCb(cb);
}

void radio_setStartFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.startFrame_cb = cb;
}

void radio_setEndFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.endFrame_cb = cb;
}

//===== reset

void radio_reset() {
}

//===== timer

void radio_startTimer(PORT_TIMER_WIDTH period) {
   radiotimer_start(period);
}

PORT_TIMER_WIDTH radio_getTimerValue() {
   return radiotimer_getValue();
}

void radio_setTimerPeriod(PORT_TIMER_WIDTH period) {
   radiotimer_setPeriod(period);
}

PORT_TIMER_WIDTH radio_getTimerPeriod() {
   return radiotimer_getPeriod();
}

//===== RF admin

void radio_setFrequency(uint8_t frequency) {
}

void radio_rfOn() {
}

void radio_rfOff() {
}

//===== TX

void radio_loadPacket(uint8_t* packet, uint16_t len) {
}

void radio_txEnable() {
}

void radio_txNow() {
}

//===== RX

void radio_rxEnable() {
}

void radio_rxNow() {
}

void radio_getReceivedFrame(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                             int8_t* pRssi,
                            uint8_t* pLqi,
                               bool* pCrc) {
}

//=========================== private =========================================


//=========================== callbacks =======================================

//=========================== interrupt handlers ==============================

kick_scheduler_t radio_isr() {
   return DO_NOT_KICK_SCHEDULER;
}

void radio_isr_internal(void) {
}

void radio_error_isr(void){
}
