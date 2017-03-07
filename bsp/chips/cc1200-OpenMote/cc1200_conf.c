/**
 * Author: Pere Tuset (peretuset@openmote.com)
           Jonathan Muñon (jonathan.munoz@inria.fr)
 * Date:   May 2016
 * Description: CC1200-specific definition of the "radio" bsp module.
 */

#include "cc1200.h"
#include "cc1200_regs.h"

//=========================== defines =========================================

/* Base frequency in kHz */
#define RF_CFG_CHAN_CENTER_F0           ( 863125 )

/* Channel spacing in kHz */
#define RF_CFG_CHAN_SPACING             ( 200 )

/* The minimum channel */
#define RF_CFG_MIN_CHANNEL              ( 0 )

/* The maximum channel */
#define RF_CFG_MAX_CHANNEL              ( 33 )

/* The maximum output power in dBm */
#define RF_CFG_MIN_TXPOWER              ( 0 )

/* The maximum output power in dBm */
#define RF_CFG_MAX_TXPOWER              ( 14 )

/* The carrier sense level used for CCA in dBm */
#define RF_CFG_CCA_THRESHOLD            ( -91 )

//=========================== variables =======================================

/**
 * CC1200 configuration for IEEE 802.15.4
 * Modulation format = 2-FSK
 * Whitening = false
 * Packet length = 255
 * Packet length mode = Variable
 * Packet bit length = 0
 * Symbol rate = 250
 * Deviation = 124.816895
 * Carrier frequency = 867.999878
 * Device address = 0
 * Manchester enable = false
 * Address config = No address check
 * Bit rate = 250
 * RX filter BW = 833.333333
 */
static const cc1200_register_settings_t cc1200_register_settings [] = {
  {CC1200_IOCFG2,            0x06},
  {CC1200_SYNC_CFG1,         0xA8},
  {CC1200_SYNC_CFG0,         0x13},
  {CC1200_DEVIATION_M,       0x99},
  {CC1200_MODCFG_DEV_E,      0x05},
  {CC1200_DCFILT_CFG,        0x26},
  {CC1200_PREAMBLE_CFG0,     0x8A},
  {CC1200_IQIC,              0x00},
  {CC1200_CHAN_BW,           0x02},
  {CC1200_MDMCFG1,           0x42},
  {CC1200_MDMCFG0,           0x05},
  {CC1200_SYMBOL_RATE2,      0xB9},
  {CC1200_SYMBOL_RATE1,      0x99},
  {CC1200_SYMBOL_RATE0,      0x9A},
  {CC1200_AGC_REF,           0x2F},
  {CC1200_AGC_CS_THR,        0xEC},
  {CC1200_AGC_CFG1,          0x16},
  {CC1200_AGC_CFG0,          0x84},
  {CC1200_FIFO_CFG,          0x00},
  {CC1200_FS_CFG,            0x12},
  {CC1200_PKT_CFG2,          0x00},
  {CC1200_PKT_CFG0,          0x20},
  {CC1200_PA_CFG1,           0x6C},
  {CC1200_PKT_LEN,           0xFF},
  {CC1200_IF_MIX_CFG,        0x18},
  {CC1200_TOC_CFG,           0x03},
  {CC1200_MDMCFG2,           0x00},
  {CC1200_FREQ2,             0x56},
  {CC1200_FREQ1,             0xCC},
  {CC1200_FREQ0,             0xCC},
  {CC1200_IF_ADC1,           0xEE},
  {CC1200_IF_ADC0,           0x10},
  {CC1200_FS_DIG1,           0x04},
  {CC1200_FS_DIG0,           0x50},
  {CC1200_FS_CAL1,           0x40},
  {CC1200_FS_CAL0,           0x0E},
  {CC1200_FS_DIVTWO,         0x03},
  {CC1200_FS_DSM0,           0x33},
  {CC1200_FS_DVC1,           0xF7},
  {CC1200_FS_DVC0,           0x0F},
  {CC1200_FS_PFD,            0x00},
  {CC1200_FS_PRE,            0x6E},
  {CC1200_FS_REG_DIV_CML,    0x1C},
  {CC1200_FS_SPARE,          0xAC},
  {CC1200_FS_VCO0,           0xB5},
  {CC1200_IFAMP,             0x0D},
  {CC1200_XOSC5,             0x0E},
  {CC1200_XOSC1,             0x03},
};

const cc1200_rf_cfg_t cc1200_rf_cfg = {
  .register_settings = cc1200_register_settings,
  .size_of_register_settings = sizeof(cc1200_register_settings),
  .chan_center_freq0 = RF_CFG_CHAN_CENTER_F0,
  .chan_spacing = RF_CFG_CHAN_SPACING,
  .min_channel = RF_CFG_MIN_CHANNEL,
  .max_channel = RF_CFG_MAX_CHANNEL,
  .min_txpower = RF_CFG_MIN_TXPOWER,
  .max_txpower = RF_CFG_MAX_TXPOWER,
  .cca_threshold = RF_CFG_CCA_THRESHOLD,
};

//=========================== prototypes ======================================

//=========================== public ==========================================

//====================== private =========================

//====================== callbacks =======================
