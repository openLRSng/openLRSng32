#pragma once
// OpenLRSng binding



// Factory setting values, modify via the CLI

//####### RADIOLINK RF POWER (beacon is always 100/13/1.3mW) #######
// 7 == 100mW (or 1000mW with M3)
// 6 == 50mW (use this when using booster amp), (800mW with M3)
// 5 == 25mW
// 4 == 13mW
// 3 == 6mW
// 2 == 3mW
// 1 == 1.6mW
// 0 == 1.3mW
#define DEFAULT_RF_POWER 7

#define DEFAULT_CARRIER_FREQUENCY 435000000  // Hz  startup frequency
#define DEFAULT_CHANNEL_SPACING 5 // 50kHz
#define DEFAULT_HOPLIST 22,10,19,34,49,41
#define DEFAULT_RF_MAGIC 0xDEADFEED

//  0 -- 4800bps, best range
//  1 -- 9600bps, medium range
//  2 -- 19200bps, medium range
#define DEFAULT_DATARATE 2

// BIND FLAGS
#define TELEMETRY_ENABLED 0x08
#define FRSKY_ENABLED     0x10
#define CHANNELS_4_4  1
#define CHANNELS_8    2
#define CHANNELS_8_4  3
#define CHANNELS_12   4
#define CHANNELS_12_4 5
#define CHANNELS_16   6

// RX flags
#define FAILSAFE_NOPPM    0x01
#define FAILSAFE_NOPWM    0x02
#define PPM_MAX_8CH       0x04
#define ALWAYS_BIND       0x08

#define DEFAULT_FLAGS CHANNELS_8

// helpper macro for European PMR channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch1-ch8

// helpper macro for US FRS channels 1-7
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch1-ch7

#define DEFAULT_BEACON_FREQUENCY 0 // disable beacon
#define DEFAULT_BEACON_DEADTIME 30 // time to wait until go into beacon mode (s)
#define DEFAULT_BEACON_INTERVAL 10 // interval between beacon transmits (s)

#define MIN_DEADTIME 10
#define MAX_DEADTIME 255

#define MIN_INTERVAL 5
#define MAX_INTERVAL 255

#define BINDING_POWER     0x00 // 1 mW
#define BINDING_VERSION   7

//#define EEPROM_OFFSET     0x00
//#define EEPROM_RX_OFFSET  0x40 // RX specific config struct

#define TELEMETRY_PACKETSIZE 9

#define BIND_MAGIC (0xDEC1BE15 + BINDING_VERSION)

extern uint8_t default_hop_list[];

// HW frequency limits
#if (defined RFMXX_868)
#  define MIN_RFM_FREQUENCY 848000000
#  define MAX_RFM_FREQUENCY 888000000
#  define DEFAULT_CARRIER_FREQUENCY 868000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 868000000 // Hz
#elif (defined RFMXX_915)
#  define MIN_RFM_FREQUENCY 895000000
#  define MAX_RFM_FREQUENCY 935000000
#  define DEFAULT_CARRIER_FREQUENCY 915000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 915000000 // Hz
#else
#  define MIN_RFM_FREQUENCY 413000000
#  define MAX_RFM_FREQUENCY 463000000
#  define DEFAULT_CARRIER_FREQUENCY 435000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 435000000 // Hz
#endif

#define MAXHOPS 24

struct __attribute__((__packed__)) bind_data {
	  uint8_t version;
	  uint32_t serial_baudrate;
	  uint32_t rf_frequency;
	  uint32_t rf_magic;
	  uint8_t rf_power;
	  uint8_t rf_channel_spacing;
	  uint8_t hopchannel[MAXHOPS];
	  uint8_t modem_params;
	  uint8_t flags;
	};

extern struct bind_data bind_data;


struct rfm22_modem_regs {
  uint32_t bps;
  uint8_t  r_1c, r_1d, r_1e, r_20, r_21, r_22, r_23, r_24, r_25, r_2a, r_6e, r_6f, r_70, r_71, r_72;
};

extern struct rfm22_modem_regs modem_params[];

#define DATARATE_COUNT (sizeof(modem_params)/sizeof(modem_params[0]))

extern struct rfm22_modem_regs bind_params;

#define PINMAP_PPM  0x20
#define PINMAP_RSSI 0x21
#define PINMAP_SDA  0x22
#define PINMAP_SCL  0x23
#define PINMAP_RXD  0x24
#define PINMAP_TXD  0x25
#define PINMAP_ANALOG 0x26

struct __attribute__((__packed__)) rx_config {
  uint8_t  rx_type; // RX type filled in by RX, do not change
  uint8_t  pinMapping[13];
  uint8_t  flags;
  uint8_t  RSSIpwm;
  uint32_t beacon_frequency;
  uint16_t  beacon_deadtime;
  uint8_t  beacon_interval;
  uint16_t minsync;
  uint8_t  failsafe_delay;
};

extern struct rx_config rx_config;

uint8_t getPacketSize();
uint8_t getChannelCount();
uint32_t getInterval();
int16_t bindReadEeprom();
void bindWriteEeprom(void);
void bindInitDefaults(void);
void bindRandomize(void);
void rxInitDefaults();
void rxWriteEeprom();
void rxReadEeprom();
void printRXconf();

