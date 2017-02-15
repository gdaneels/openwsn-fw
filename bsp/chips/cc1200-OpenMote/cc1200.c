/**
 * Author: Pere Tuset (peretuset@openmote.com)
           Jonathan Muñon (jonathan.munoz@inria.fr)
 * Date:   May 2016
 * Description: CC1200-specific definition of the "radio" bsp module.
 */

#include <string.h>

#include "cc1200.h"
#include "cc1200_regs.h"
#include "cc1200_arch.h"

#include "gpio.h"
#include <source/gpio.h>

//=========================== defines =========================================

/* 868.325 MHz */
#define CC1200_DEFAULT_CHANNEL          ( 26 )

/* Calibration is done by hand */
#define CC1200_AUTOCAL                  ( 0 )

/*
 * Divider + multiplier for calculation of FREQ registers
 * f * 2^16 * 4 / 40000 = f * 2^12 / 625 (no overflow up to frequencies of
 * 1048.576 MHz using uint32_t)
 */
#define XTAL_FREQ_KHZ                   ( 40000 )
#define LO_DIVIDER                      ( 4 )
#define FREQ_DIVIDER                    ( 625 )
#define FREQ_MULTIPLIER                 ( 4096 )

#define PHR_LEN                         ( 2 )
#define APPENDIX_LEN                    ( 2 )

#define CC1200_RSSI_OFFSET              ( -81 )
#define CC1200_FREQ_OFFSET              ( 0 )

/* Packet start on rising edge of GPIO2 and is read out on falling edge of GPIO2 */
#define GPIO2_IOCFG                     CC1200_IOCFG_PKT_SYNC_RXTX

#define CC1200_GPIO2_PORT               ( GPIO_D_PORT )
#define CC1200_GPIO2_PIN                ( GPIO_PIN_1 )
#define CC1200_GPIO2_PIN_NR             ( 1 )

#define STATE_USES_MARC_STATE           ( 0 )
#if STATE_USES_MARC_STATE
/* We use the MARC_STATE register to poll the chip's status */
#define STATE_IDLE                      CC1200_MARC_STATE_IDLE
#define STATE_RX                        CC1200_MARC_STATE_RX
#define STATE_TX                        CC1200_MARC_STATE_TX
#define STATE_RX_FIFO_ERROR             CC1200_MARC_STATE_RX_FIFO_ERR
#define STATE_TX_FIFO_ERROR             CC1200_MARC_STATE_TX_FIFO_ERR
#else
/* We use the status byte read out using a NOP strobe */
#define STATE_IDLE                      CC1200_STATUS_BYTE_IDLE
#define STATE_RX                        CC1200_STATUS_BYTE_RX
#define STATE_TX                        CC1200_STATUS_BYTE_TX
#define STATE_FSTXON                    CC1200_STATUS_BYTE_FSTXON
#define STATE_CALIBRATE                 CC1200_STATUS_BYTE_CALIBRATE
#define STATE_SETTLING                  CC1200_STATUS_BYTE_SETTLING
#define STATE_RX_FIFO_ERR               CC1200_STATUS_BYTE_RX_FIFO_ERR
#define STATE_TX_FIFO_ERR               CC1200_STATUS_BYTE_TX_FIFO_ERR
#endif /* #if STATE_USES_MARC_STATE */

/* Radio was initialized (= init() was called) */
#define RF_INITIALIZED                  0x01
/* The radio is on (= not in standby) */
#define RF_ON                           0x02
/* An incoming packet was detected (at least payload length was received */
#define RF_RX_PROCESSING_PKT            0x04
/* TX is ongoing */
#define RF_TX_ACTIVE                    0x08
/* Channel update required */
#define RF_UPDATE_CHANNEL               0x10
/* SPI was locked when calling RX interrupt, let the pollhandler do the job */
#define RF_POLL_RX_INTERRUPT            0x20

/* Bit Masks for the last byte in the RX FIFO */
#define CRC_BIT_MASK 0x80
#define LQI_BIT_MASK 0x7F

//=========================== variables =======================================

// TODO: Clean up this temporary code that is required to handle the interrupt callback
#include "radio.h"
#include "radiotimer.h"
typedef struct {
   radiotimer_capture_cbt    startFrame_cb;
   radiotimer_capture_cbt    endFrame_cb;
   radio_state_t             state; 
} radio_vars_t;
extern radio_vars_t radio_vars;

extern const cc1200_rf_cfg_t cc1200_rf_cfg;

static uint8_t rf_flags = 0;

//=========================== prototypes ======================================

static uint8_t cc1200_strobe(uint8_t strobe);
static uint8_t cc1200_single_read(uint16_t address);
static void cc1200_burst_read(uint16_t address, uint8_t *data, uint8_t length);
static uint8_t cc1200_single_write(uint16_t address, uint8_t value);
static void cc1200_burst_write(uint16_t address, const uint8_t *data, uint8_t length);
static uint8_t cc1200_state(void);
static void cc1200_write_register_settings(const cc1200_register_settings_t* settings, uint16_t size);

//=========================== public ==========================================

/**
 * Initialize the CC1200.
 */
void cc1200_init(void) {
  if(!(rf_flags & RF_INITIALIZED)) {
      /* Initialize the low-level */
      cc1200_arch_init();

      /* Initialize the GPIOs */
      gpio_config_input(CC1200_GPIO2_PORT, CC1200_GPIO2_PIN, GPIO_BOTH_EDGES);
      gpio_register_callback(CC1200_GPIO2_PORT, CC1200_GPIO2_PIN_NR, &cc1200_gpio2_interrupt);
      gpio_enable_interrupt(CC1200_GPIO2_PORT, CC1200_GPIO2_PIN);

      /* Write the initial configuration */
      cc1200_configure();

      /* Set output power */
      cc1200_set_tx_power(cc1200_rf_cfg.max_txpower);

      /* Set default channel. This will also force initial calibration! */
      cc1200_set_channel(CC1200_DEFAULT_CHANNEL);

      /* We are on + initialized at this point */
      rf_flags |= (RF_INITIALIZED + RF_ON);

      /* Turn it off to conserve energy */
      cc1200_off();
  }
}

/**
 * Turn the CC1200 on.
 */
bool cc1200_on(void) {
  /* Don't turn on if we are on already */
  if (!(rf_flags & RF_ON)) {
    /* Wake-up procedure, wait for GPIO0 to de-assert (CHIP_RDYn) */
    cc1200_arch_spi_select();

    /* Wait until CHIP_RDYn signal on GPIO 0 is not low */
    while (cc1200_arch_gpio0_read() != 0) {
        cc1200_arch_clock_delay(100);
    }

    /* Wake-up procedure completed */
    cc1200_arch_spi_deselect();

    /* After a reset the chip is in IDLE mode */
    rf_flags |= RF_ON;
  }

  return true;
}

/**
 * Turn the radio off.
 */
bool cc1200_off(void) {
  /* Don't turn off if we are off already */
  if (rf_flags & RF_ON) {
    /* Put radio in idle mode */
    cc1200_idle();

    /* Put the radio if off */
    cc1200_strobe(CC1200_SPWD);

    /* Clear all but the initialized flag */
    rf_flags = RF_INITIALIZED;
  }

  return true;
}

/**
 * Put the radio in IDLE mode.
 */
void cc1200_idle(void) {
  uint8_t state;

  /* Read CC1200 state */
  state = cc1200_state();

  /* Manage special cases */
  if (state == STATE_IDLE) {
    return;
  } else if(state == STATE_RX_FIFO_ERR) {
    cc1200_strobe(CC1200_SFRX);
  } else if(state == STATE_TX_FIFO_ERR) {
    cc1200_strobe(CC1200_SFTX);
  }

  /* Put the CC1200 in IDLE mode */
  cc1200_strobe(CC1200_SIDLE);

  /* Busy-wait until the CC1200 is in IDLE mode */
  while (cc1200_state() != STATE_IDLE) {
      cc1200_arch_clock_delay(100);
  }
}

/**
 * Reset the radio.
 */
void cc1200_reset(void) {
  cc1200_arch_spi_select();

  /* Send a reset strobe */
  cc1200_arch_spi_rw_byte(CC1200_SRES);

  /* Busy-wait until reset is complete */
  cc1200_arch_clock_delay(150);

  cc1200_arch_spi_deselect();
}

/**
 * Configure the radio.
 */
void cc1200_configure(void) {
    /* Make sure the CC1200 is in default state */
    cc1200_reset();

    /* Write configuration to the CC1200 */
    cc1200_write_register_settings(cc1200_rf_cfg.register_settings,
                                   cc1200_rf_cfg.size_of_register_settings);

    /* Write frequency offset MSB and LSB */
    cc1200_single_write(CC1200_FREQOFF1, (uint8_t)(CC1200_FREQ_OFFSET >> 8));
    cc1200_single_write(CC1200_FREQOFF0, (uint8_t)(CC1200_FREQ_OFFSET >> 0));

    /* RSSI offset */
    cc1200_single_write(CC1200_AGC_GAIN_ADJUST, (int8_t)CC1200_RSSI_OFFSET);

    /* GPIO configuration */
    cc1200_single_write(CC1200_IOCFG2, GPIO2_IOCFG);
}

/**
 * Perform a manual calibration.
 */
void cc1200_calibrate(void) {
  uint8_t state;

  /* Perform a manual calibration */
  cc1200_strobe(CC1200_SCAL);

  /* Wait until radio is in receive mode */
  state = cc1200_state();
  while (state != STATE_CALIBRATE) {
      cc1200_arch_clock_delay(100);
      state = cc1200_state();
  }

  /* Wait until radio is in receive mode */
  while (cc1200_state() != STATE_IDLE) {
      cc1200_arch_clock_delay(100);
  }
}

/**
 * Set the transmit power.
 */
void cc1200_set_tx_power(int8_t tx_power_dbm) {
  uint8_t reg;

  /* Read the current transmit power value */
  reg = cc1200_single_read(CC1200_PA_CFG1);

  /* Update the transmit power value*/
  reg &= ~0x3F;
  reg |= ((((tx_power_dbm + 18) * 2) - 1) & 0x3F);

  /* Write the new transmit power value */
  cc1200_single_write(CC1200_PA_CFG1, reg);
}

/**
 * Put the radio in transmit mode.
 */
void cc1200_transmit(void) {
    /* Enable synthetiser */
    cc1200_strobe(CC1200_SFSTXON);

    /* Enable transmission */
    cc1200_strobe(CC1200_STX);
}

/**
 * Put the radio in receive mode.
 */
void cc1200_receive(void) {
    cc1200_on();
    cc1200_idle();

    /* Calibrate before entering RX */
    cc1200_calibrate();

    rf_flags &= ~RF_RX_PROCESSING_PKT;

    /* Empty the receive buffer */
    cc1200_strobe(CC1200_SFRX);

    /* Go to receive mode */
    cc1200_strobe(CC1200_SRX);

    /* Wait until radio is in receive mode */
    while (cc1200_state() != STATE_RX) {
        cc1200_arch_clock_delay(100);
    }
}

/**
 * Set the channel.
 */
bool cc1200_set_channel(uint8_t channel) {
  uint32_t frequency;
  bool was_off;

  if (channel < cc1200_rf_cfg.min_channel ||
      channel > cc1200_rf_cfg.max_channel) {
    return false;
  }

  /* Check if tha radio is off */
  was_off = false;

  /* If it is off, turn it on */
  if (was_off) {
      cc1200_on();
  }

  /* Put the CC1200 in idle mode */
  cc1200_idle();

  /* Calculate the channel frequency */
  frequency = cc1200_rf_cfg.chan_center_freq0 + channel * cc1200_rf_cfg.chan_spacing;
  frequency *= FREQ_MULTIPLIER;
  frequency /= FREQ_DIVIDER;

  /* Write the CC1200 registers */
  cc1200_single_write(CC1200_FREQ0, ((uint8_t *)&frequency)[0]);
  cc1200_single_write(CC1200_FREQ1, ((uint8_t *)&frequency)[1]);
  cc1200_single_write(CC1200_FREQ2, ((uint8_t *)&frequency)[2]);

  /* Turn on RX again unless we turn off anyway */
  if (!was_off) {
      cc1200_calibrate();
  }

  /* If it was off, turn it off again */
  if (was_off) {
      cc1200_off();
  }

  return true;
}

/**
 * Load a packet into the radio.
 */
void cc1200_load_packet(uint8_t* buffer, uint16_t length) {
    cc1200_strobe(CC1200_SFTX);

    // Busy-wait until the CC1200 is in IDLE mode
    while (cc1200_state() != STATE_IDLE) {
        cc1200_arch_clock_delay(10);
    }

    cc1200_single_write(CC1200_TXFIFO, length);
    cc1200_burst_write(CC1200_TXFIFO, buffer, length);
}

/**
 * Get a packet that has been received by the radio.
 */
void cc1200_get_packet(uint8_t* buffer,
                       uint8_t* lenRead,
                       uint8_t  maxLength,
                        int8_t* rssi,
                       uint8_t* lqi,
                          bool* crc) {

    uint8_t crc_corr;
    uint16_t length = cc1200_single_read(CC1200_NUM_RXBYTES);

    if ((length >= 3) && (length <= 1+125+2))
    {
        // Read the length byte
        uint8_t len = cc1200_single_read(CC1200_RXFIFO);
        if (len == length - 3)
        {
            // Read the received packet
            cc1200_burst_read(CC1200_RXFIFO, buffer, len);

            // Read the 2 bytes FCS
            *rssi    = cc1200_single_read(CC1200_RXFIFO);
            crc_corr = cc1200_single_read(CC1200_RXFIFO);
            *lqi     = crc_corr & LQI_BIT_MASK;
            *crc     = crc_corr & CRC_BIT_MASK;
            *lenRead = len;
        }
    }

    // Flush the buffer and listen for the next packet
    cc1200_receive();
}

//====================== private =========================

/**
 * Send a command strobe.
 */
static uint8_t cc1200_strobe(uint8_t strobe) {
  uint8_t ret;
  cc1200_arch_spi_select();
  ret = cc1200_arch_spi_rw_byte(strobe);
  cc1200_arch_spi_deselect();
  return ret;
}

/**
 * Read a single byte from the specified address.
 */
static uint8_t cc1200_single_read(uint16_t address) {
  uint8_t ret;

  cc1200_arch_spi_select();

  if (CC1200_IS_EXTENDED_ADDR(address)) {
    cc1200_arch_spi_rw_byte(CC1200_EXTENDED_READ_CMD);
    cc1200_arch_spi_rw_byte((uint8_t)address);
  } else {
    cc1200_arch_spi_rw_byte(address | CC1200_READ_BIT);
  }
  ret = cc1200_arch_spi_rw_byte(0);

  cc1200_arch_spi_deselect();

  return ret;
}

/**
 * Read a burst of bytes starting at the specified address.
 */
static void cc1200_burst_read(uint16_t address, uint8_t *data, uint8_t length) {
  cc1200_arch_spi_select();

  if (CC1200_IS_EXTENDED_ADDR(address)) {
    cc1200_arch_spi_rw_byte(CC1200_EXTENDED_BURST_READ_CMD);
    cc1200_arch_spi_rw_byte((uint8_t)address);
  } else {
    cc1200_arch_spi_rw_byte(address | CC1200_READ_BIT | CC1200_BURST_BIT);
  }

  cc1200_arch_spi_rw(data, NULL, length);
  cc1200_arch_spi_deselect();
}

/**
 * Write a single byte to the specified address.
 */
static uint8_t cc1200_single_write(uint16_t address, uint8_t value) {
  uint8_t ret;

  cc1200_arch_spi_select();

  if (CC1200_IS_EXTENDED_ADDR(address)) {
    cc1200_arch_spi_rw_byte(CC1200_EXTENDED_WRITE_CMD);
    cc1200_arch_spi_rw_byte((uint8_t)address);
  } else {
    cc1200_arch_spi_rw_byte(address | CC1200_WRITE_BIT);
  }
  ret = cc1200_arch_spi_rw_byte(value);

  cc1200_arch_spi_deselect();

  return ret;
}

/**
 * Write a burst of bytes starting at the specified address.
 */
static void cc1200_burst_write(uint16_t address, const uint8_t *data, uint8_t length) {
  cc1200_arch_spi_select();

  if (CC1200_IS_EXTENDED_ADDR(address)) {
    cc1200_arch_spi_rw_byte(CC1200_EXTENDED_BURST_WRITE_CMD);
    cc1200_arch_spi_rw_byte((uint8_t)address);
  } else {
    cc1200_arch_spi_rw_byte(address | CC1200_WRITE_BIT | CC1200_BURST_BIT);
  }

  cc1200_arch_spi_rw(NULL, data, length);

  cc1200_arch_spi_deselect();
}

static uint8_t cc1200_state(void) {
#if STATE_USES_MARC_STATE
  return cc1200_single_read(CC1200_MARCSTATE) & 0x1F;
#else
  return cc1200_strobe(CC1200_SNOP) & 0x70;
#endif
}

/**
 * Write a list of register settings.
 */
void cc1200_write_register_settings(const cc1200_register_settings_t* settings, uint16_t size) {
  uint16_t i = size / sizeof(cc1200_register_settings_t);

  if (settings != NULL) {
    while (i--) {
      cc1200_single_write(settings->address, settings->value);
      settings++;
    }
  }
}

//====================== callbacks =======================

void cc1200_gpio0_interrupt(void) {
}

void cc1200_gpio2_interrupt(void) {
    bool high = gpio_read(CC1200_GPIO2_PORT, CC1200_GPIO2_PIN);
    if (high) {
        if (radio_vars.startFrame_cb != NULL) {
            radio_vars.startFrame_cb(radiotimer_getCapturedTime());
        }
    }
    else {
        if (radio_vars.endFrame_cb != NULL) {
            radio_vars.endFrame_cb(radiotimer_getCapturedTime());
        }
    }
}

void cc1200_gpio3_interrupt(void) {
}
