/**
 * Author: Pere Tuset (peretuset@openmote.com)
           Jonathan Muñon (jonathan.munoz@inria.fr)
 * Date:   May 2016
 * Description: CC1200-specific definition of the "radio" bsp module.
 */


#include "board.h"
#include "radio.h"
#include "cc1200.h"
#include "debugpins.h"
#include "radiotimer.h"
#include "leds.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   radiotimer_capture_cbt    startFrame_cb;
   radiotimer_capture_cbt    endFrame_cb;
   radio_state_t             state; 
} radio_vars_t;

radio_vars_t radio_vars;

//=========================== prototypes ======================================

void radio_spiStrobe     (uint8_t strobe, cc1200_status_t* statusRead);
void radio_spiWriteReg   (uint8_t reg,    cc1200_status_t* statusRead, uint8_t  regValueToWrite);
void radio_spiReadReg    (uint8_t reg,    cc1200_status_t* statusRead, uint8_t* regValueRead);
void radio_spiWriteTxFifo(                cc1200_status_t* statusRead, uint8_t* bufToWrite, uint8_t  lenToWrite);
void radio_spiReadRxFifo (                cc1200_status_t* statusRead, uint8_t* bufRead,    uint8_t* lenRead, uint8_t maxBuf);

//=========================== public ==========================================

//==== admin

void radio_init(void) {
  cc1200_init();
}

void radio_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_setOverflowCb(cb);
}

void radio_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_setCompareCb(cb);
}

void radio_setStartFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.startFrame_cb  = cb;
}

void radio_setEndFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.endFrame_cb    = cb;
}

//==== reset

void radio_reset(void) {
    cc1200_reset();
}

//==== timer

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

//==== RF admin

void radio_setFrequency(uint8_t frequency) {
    cc1200_set_channel(frequency);
}

void radio_rfOn(void) {
    cc1200_on();
}

void radio_rfOff(void) {

    // change state
    radio_vars.state = RADIOSTATE_TURNING_OFF;
    cc1200_off();

    // wiggle debug pin
    debugpins_radio_clr();
    leds_radio_off();

    // change state
    radio_vars.state = RADIOSTATE_RFOFF;
}

//==== TX

void radio_loadPacket(uint8_t* packet, uint16_t len) {
    cc1200_load_packet(packet, len);
}

void radio_txEnable() {
    // wiggle debug pin
    debugpins_radio_set();
    leds_radio_on();

    cc1200_calibrate();
}

void radio_txNow(void) {
    cc1200_transmit();
}

//==== RX

void radio_rxEnable(void) {
    // wiggle debug pin
    debugpins_radio_set();
    leds_radio_on();
}

void radio_rxNow(void) {
    cc1200_receive();
}

void radio_getReceivedFrame(uint8_t* bufRead,
                            uint8_t* lenRead,
                            uint8_t maxBufLen,
                             int8_t* rssi,
                            uint8_t* lqi,
                               bool* crc) {

    cc1200_get_packet(bufRead, lenRead, maxBufLen, rssi, lqi, crc);
}

//====================== private =========================

//====================== callbacks =======================
