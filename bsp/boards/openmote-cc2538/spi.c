/**
 * Author: Pere Tuset (peretuset@openmote.com)
 *         Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 * Date:   May 2016
 * Description: CC2538-specific definition of the "spi" bsp module.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <headers/hw_ints.h>
#include <headers/hw_ioc.h>
#include <headers/hw_memmap.h>
#include <headers/hw_types.h>
#include <headers/hw_ssi.h>

#include <source/ssi.h>
#include <source/gpio.h>
#include <source/interrupt.h>
#include <source/sys_ctrl.h>
#include <source/ioc.h>

#include "spi.h"
#include "board.h"
#include "debugpins.h"

//=========================== defines =========================================

#define SPI_PERIPHERAL          ( SYS_CTRL_PERIPH_SSI0 )
#define SPI_BASE                ( SSI0_BASE )
#define SPI_CLOCK               ( SSI_CLOCK_PIOSC )
#define SPI_INT                 ( INT_SSI0 )
#define SPI_MODE                ( SSI_MODE_MASTER )
#define SPI_PROTOCOL            ( SSI_FRF_MOTO_MODE_0 )
#define SPI_DATAWIDTH           ( 8 )
#define SPI_BAUDRATE            ( 8000000 )

#define SPI_MISO_BASE           ( GPIO_A_BASE )
#define SPI_MISO_PIN            ( GPIO_PIN_4 )
#define SPI_MISO_IOC            ( IOC_SSIRXD_SSI0 )
#define SPI_MOSI_BASE           ( GPIO_A_BASE )
#define SPI_MOSI_PIN            ( GPIO_PIN_5 )
#define SPI_MOSI_IOC            ( IOC_MUX_OUT_SEL_SSI0_TXD )
#define SPI_CLK_BASE            ( GPIO_A_BASE )
#define SPI_CLK_PIN             ( GPIO_PIN_2 )
#define SPI_CLK_IOC             ( IOC_MUX_OUT_SEL_SSI0_CLKOUT )

//=========================== variables =======================================

typedef struct {
   uint8_t* pNextTxByte;
   uint8_t  numTxedBytes;
   uint8_t  txBytesLeft;
   uint8_t* pNextRxByte;
   uint8_t  maxRxBytes;
   uint8_t  busy;
} spi_vars_t;

spi_vars_t spi_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void spi_init(void) {
  // Reset peripheral previous to configuring it
  SSIDisable(SPI_BASE);

  // Set IO clock as SPI0 clock source
  SSIClockSourceSet(SPI_BASE, SPI_CLOCK);

  // Configure the MISO, MOSI, CLK and nCS pins as peripheral
  IOCPinConfigPeriphInput(SPI_MISO_BASE, SPI_MISO_PIN, SPI_MISO_IOC);
  IOCPinConfigPeriphOutput(SPI_MOSI_BASE, SPI_MOSI_PIN, SPI_MOSI_IOC);
  IOCPinConfigPeriphOutput(SPI_CLK_BASE, SPI_CLK_PIN, SPI_CLK_IOC);

  // Configure MISO, MOSI, CLK and nCS GPIOs
  GPIOPinTypeSSI(SPI_MISO_BASE, SPI_MISO_PIN);
  GPIOPinTypeSSI(SPI_MOSI_BASE, SPI_MOSI_PIN);
  GPIOPinTypeSSI(SPI_CLK_BASE, SPI_CLK_PIN);

  // Configure the SPI0 clock
  SSIConfigSetExpClk(SPI_BASE, SysCtrlIOClockGet(), SPI_PROTOCOL, \
                     SPI_MODE, SPI_BAUDRATE, SPI_DATAWIDTH);

  // Enable the SPI0 module
  SSIEnable(SPI_BASE);
}

void spi_setCallback(spi_cbt cb) {
}

void spi_txrx(uint8_t*     bufTx,
              uint8_t      lenbufTx,
              spi_return_t returnType,
              uint8_t*     bufRx,
              uint8_t      maxLenBufRx,
              spi_first_t  isFirst,
              spi_last_t   isLast) {
  uint32_t data;

  // Register SPI frame to send
  spi_vars.pNextTxByte  = bufTx;
  spi_vars.numTxedBytes = 0;
  spi_vars.txBytesLeft  = lenbufTx;
  spi_vars.pNextRxByte  = bufRx;
  spi_vars.maxRxBytes   = maxLenBufRx;

  // SPI is now busy
  spi_vars.busy = 1;

  // Wait until all bytes are transmitted
  while (spi_vars.txBytesLeft > 0) {
    // Push a byte
    SSIDataPut(SPI_BASE, *spi_vars.pNextTxByte);

    // Wait until it is complete
    while (SSIBusy(SPI_BASE))
      ;

    // Save the byte just received in the RX buffer
    switch (returnType) {
        uint32_t data;
        case SPI_FIRSTBYTE:
            if (spi_vars.numTxedBytes==0) {
                SSIDataGet(SPI_BASE, &data);
                *spi_vars.pNextRxByte = data;
            }
            break;
        case SPI_BUFFER:
            SSIDataGet(SPI_BASE, &data);
            *spi_vars.pNextRxByte = data;
            spi_vars.pNextRxByte++;
            break;
        case SPI_LASTBYTE:
            SSIDataGet(SPI_BASE, &data);
            *spi_vars.pNextRxByte = data;
            break;
    }

    // one byte less to go
    spi_vars.pNextTxByte++;
    spi_vars.numTxedBytes++;
    spi_vars.txBytesLeft--;
  }

  // SPI is not busy anymore
  spi_vars.busy = 0;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

kick_scheduler_t spi_isr(void) {
  return DO_NOT_KICK_SCHEDULER;
}
