/**
 * Author: Pere Tuset (peretuset@uoc.edu)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Alda Xhafa (axhafa@uoc.edu)
 * Date:   December 2016
 * Description: 
 */

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "uart.h"
#include "board.h"
#include "debugpins.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   uart_tx_cbt txCb;
   uart_rx_cbt rxCb;
} uart_vars_t;

uart_vars_t uart_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void uart_init() { 
   // Reset local variables
   memset(&uart_vars,0,sizeof(uart_vars_t));
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) {
    uart_vars.txCb = txCb;
    uart_vars.rxCb = rxCb;
}

void uart_enableInterrupts(){
    
}

void uart_disableInterrupts(){
    
}

void uart_clearRxInterrupts(){
    
}

void uart_clearTxInterrupts(){
    
}

void  uart_writeByte(uint8_t byteToWrite){

}

uint8_t uart_readByte(){
	return 0x00;
}

//=========================== interrupt handlers ==============================

kick_scheduler_t uart_rx_isr() {
	return DO_NOT_KICK_SCHEDULER;
}
