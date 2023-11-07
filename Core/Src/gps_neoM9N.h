/*
 * gps_neoM9N.h
 *
 *  Created on: Feb 7, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_GPS_NEOM9N_H_
#define SRC_GPS_NEOM9N_H_

#include <stdint.h>
#include "time.h"
#include "stm32l4xx_hal.h"
#include "uart.h"
#include "main.h"

#define UBLOX_M9N               (0x42 << 1)


#define UBX_SYNCH_1     0xb5
#define UBX_SYNCH_2     0x62
#define UBX_CLASS_NAV   0x01  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
#define UBX_CLASS_RXM   0x02  // Receiver Manager Messages: Satellite Status, RTC Status
#define UBX_CLASS_INF   0x04  // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
#define UBX_CLASS_ACK   0x05  // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
#define UBX_CLASS_CFG   0x06  // Configuration Input Messages: Configure the receiver.
#define UBX_CLASS_ID    0x86
#define UBX_CLASS_UPD   0x09  // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
#define UBX_CLASS_MON   0x0A  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
#define UBX_CLASS_TIM   0x0D  // Timing Messages: Time Pulse Output, Time Mark Results
#define UBX_CLASS_MGA   0x13  // Multiple GNSS Assistance Messages: Assistance data for various GNSS
#define UBX_CLASS_LOG   0x21  // Logging Messages: Log creation, deletion, info and retrieval
#define UBX_CLASS_SEC   0x27  // Security Feature Messages
#define UBX_CLASS_NMEA  0xF0 // NMEA Strings: standard NMEA strings
#define UBX_CLASS_PUBX  0xF1 // Proprietary NMEA-format messages defined by u-blox

#define UBX_CFG_ANT         0x13        // Antenna Control Settings. Used to configure the antenna control settings
#define UBX_CFG_BATCH       0x93      // Get/set data batching configuration.
#define UBX_CFG_CFG         0x09        // Clear, Save, and Load Configurations. Used to save current configuration
#define UBX_CFG_DAT         0x06        // Set User-defined Datum or The currently defined Datum
#define UBX_CFG_DGNSS       0x70      // DGNSS configuration
#define UBX_CFG_ESFALG      0x56     // ESF alignment
#define UBX_CFG_ESFA        0x4C       // ESF accelerometer
#define UBX_CFG_ESFG        0x4D       // ESF gyro
#define UBX_CFG_GEOFENCE    0x69   // Geofencing configuration. Used to configure a geofence
#define UBX_CFG_GNSS        0x3E       // GNSS system configuration
#define UBX_CFG_HNR         0x5C        // High Navigation Rate
#define UBX_CFG_INF         0x02        // Depending on packet length, either: poll configuration for one protocol, or information message configuration
#define UBX_CFG_ITFM        0x39       // Jamming/Interference Monitor configuration
#define UBX_CFG_LOGFILTER   0x47  // Data Logger Configuration
#define UBX_CFG_MSG         0x01        // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
#define UBX_CFG_NAV5        0x24       // Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
#define UBX_CFG_NAVX5       0x23      // Navigation Engine Expert Settings
#define UBX_CFG_NMEA        0x17       // Extended NMEA protocol configuration V1
#define UBX_CFG_ODO         0x1E        // Odometer, Low-speed COG Engine Settings
#define UBX_CFG_PM2         0x3B        // Extended power management configuration
#define UBX_CFG_PMS         0x86        // Power mode setup
#define UBX_CFG_PRT         0x00        // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
#define UBX_CFG_PWR         0x57        // Put receiver in a defined power state
#define UBX_CFG_RATE        0x08       // Navigation/Measurement Rate Settings. Used to set port baud rates.
#define UBX_CFG_RINV        0x34       // Contents of Remote Inventory
#define UBX_CFG_RST         0x04        // Reset Receiver / Clear Backup Data Structures. Used to reset device.
#define UBX_CFG_RXM         0x11        // RXM configuration
#define UBX_CFG_SBAS        0x16       // SBAS configuration
#define UBX_CFG_TMODE3      0x71     // Time Mode Settings 3. Used to enable Survey In Mode
#define UBX_CFG_TP5         0x31        // Time Pulse Parameters
#define UBX_CFG_USB         0x1B        // USB Configuration
#define UBX_CFG_VALDEL      0x8C     // Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
#define UBX_CFG_VALGET      0x8B     // Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
#define UBX_CFG_VALSET      0x8A     // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

// Class: NMEA
// The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
#define UBX_NMEA_MSB  0xF0 // All NMEA enable commands have 0xF0 as MSB. Equal to UBX_CLASS_NMEA
#define UBX_NMEA_DTM  0x0A // GxDTM (datum reference)
#define UBX_NMEA_GAQ  0x45 // GxGAQ (poll a standard message (if the current talker ID is GA))
#define UBX_NMEA_GBQ  0x44 // GxGBQ (poll a standard message (if the current Talker ID is GB))
#define UBX_NMEA_GBS  0x09 // GxGBS (GNSS satellite fault detection)
#define UBX_NMEA_GGA  0x00 // GxGGA (Global positioning system fix data)
#define UBX_NMEA_GLL  0x01 // GxGLL (latitude and long, whith time of position fix and status)
#define UBX_NMEA_GLQ  0x43 // GxGLQ (poll a standard message (if the current Talker ID is GL))
#define UBX_NMEA_GNQ  0x42 // GxGNQ (poll a standard message (if the current Talker ID is GN))
#define UBX_NMEA_GNS  0x0D // GxGNS (GNSS fix data)
#define UBX_NMEA_GPQ  0x40 // GxGPQ (poll a standard message (if the current Talker ID is GP))
#define UBX_NMEA_GQQ  0x47 // GxGQQ (poll a standard message (if the current Talker ID is GQ))
#define UBX_NMEA_GRS  0x06 // GxGRS (GNSS range residuals)
#define UBX_NMEA_GSA  0x02 // GxGSA (GNSS DOP and Active satellites)
#define UBX_NMEA_GST  0x07 // GxGST (GNSS Pseudo Range Error Statistics)
#define UBX_NMEA_GSV  0x03 // GxGSV (GNSS satellites in view)
#define UBX_NMEA_RLM  0x0B // GxRMC (Return link message (RLM))
#define UBX_NMEA_RMC  0x04 // GxRMC (Recommended minimum data)
#define UBX_NMEA_TXT  0x41 // GxTXT (text transmission)
#define UBX_NMEA_VLW  0x0F // GxVLW (dual ground/water distance)
#define UBX_NMEA_VTG  0x05 // GxVTG (course over ground and Ground speed)
#define UBX_NMEA_ZDA  0x08 // GxZDA (Time and Date)

#define VAL_RXM_PMREQ_UARTRX  0x00000008  // uartrx
#define VAL_RXM_PMREQ_EXTINT0  0x00000020 // extint0
#define VAL_RXM_PMREQ_EXTINT1  0x00000040 // extint1
#define VAL_RXM_PMREQ_SPICS  0x00000080

// Class: RXM
// The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
#define UBX_RXM_COR         0x34       // Differential correction input status
#define UBX_RXM_MEASX       0x14     // Satellite Measurements for RRLP
#define UBX_RXM_PMP         0x72       // PMP raw data (NEO-D9S) (two different versions) (packet size for version 0x01 is variable)
#define UBX_RXM_QZSSL6      0x73    // QZSSL6 data (NEO-D9C)
#define UBX_RXM_PMREQ       0x41     // Requests a Power Management task (two different packet sizes)
#define UBX_RXM_RAWX        0x15      // Multi-GNSS Raw Measurement Data
#define UBX_RXM_RLM         0x59       // Galileo SAR Short-RLM report (two different packet sizes)
#define UBX_RXM_RTCM        0x32      // RTCM input status
#define UBX_RXM_SFRBX       0x13     // Broadcast Navigation Data Subframe
#define UBX_RXM_SPARTN      0x33    // SPARTN input status
#define UBX_RXM_SPARTNKEY   0x36 // Poll/transfer dynamic SPARTN keys

#define COM_PORT_I2C     0
#define COM_PORT_UART1   1
#define COM_PORT_UART2   2
#define COM_PORT_USB     3
#define COM_PORT_SPI     4

#define COM_TYPE_UBX     0x01
#define COM_TYPE_NMEA    0x02
#define COM_TYPE_RTCM3   0x20
#define COM_TYPE_SPARTN  0x40


//UBLOX DEFAULT CONFIGURATION
#define NMEA_GGA_RATE           0x02
#define NMEA_GGL_RATE           0x00
#define NMEA_GSA_RATE           0x00
#define NMEA_GSV_RATE           0x00
#define NMEA_RMC_RATE           0x00
#define NMEA_VTG_RATE           0x00



#define UBLOX_RST_WARM_ST   0x0001
#define UBLOX_RST_HOT_ST    0x0000
#define UBLOX_RST_COLD_ST   0xFFFF

#define UBLOX_RST_START     0x09
#define UBLOX_RST_STOP      0x08
#define UBLOX_RST_RST       0x00
#define UBLOX_RST_SW_RST    0x02


#define UBLOX_PWR_MODE_CONTINUOUS 0x00
#define UBLOX_PWR_MODE_POWER_SAVE 0x01

#define GPS_LOSS_COUNT_THR          20

//#define NMEA_delimiter  0x2C

typedef struct{
    char sentence[100];
    char timestamp[9];
    long latitude;
    char NS[1];
    long longtitude;
    char EW[1];
    char quality[1];
    long altitude;
    char satellites[2];
    char HDOP[5];
    char sep[6];
}gps_data_t;

typedef enum {
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
  SFE_UBLOX_PACKET_VALIDITY_VALID,
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
  SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

typedef struct{
    uint8_t cls;
    uint8_t id;
    uint16_t len;           // Length of the payload. Does not include cls, id, or checksum bytes
    uint16_t counter;       // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
    uint16_t startingSpot;  // The counter value needed to go past before we begin recording into payload array
    uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
    uint8_t checksumA;      // Given to us from module. Checked against the rolling calculated A/B checksums.
    uint8_t checksumB;
    sfe_ublox_packet_validity_e valid;            // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
    sfe_ublox_packet_validity_e classAndIDmatch;
}messageCFG_t;

typedef enum{
	UBX_NACK,
	UBX_ACK,
	TRANS_ERROR,
	RECEIVE_ERROR,
	TRANS_OK
}UBLOX_transResult;


UBLOX_transResult ubloxInit(void);

uint8_t ublox_i2c_bus_init(void);

void ublox_tick(void);

uint16_t ubloxGetBytes(uint8_t *buf);

uint8_t ubloxRead(void);

UBLOX_transResult ubloxNmeaGGA_set_refresh_rate(uint8_t seconds);

UBLOX_transResult setPortOutput(uint8_t portSelect, uint8_t streamSettings);

UBLOX_transResult configureNMEA(uint8_t msgClass, uint8_t msgID, uint8_t rate, uint8_t portID);

UBLOX_transResult ubloxSaveConfiguration(void);

UBLOX_transResult ubloxClearConfiguration(void);

UBLOX_transResult ubloxLoadConfiguration(void);

UBLOX_transResult getPortSettings(uint8_t portID, uint8_t *rx_mes);

UBLOX_transResult sendI2Cmessage(void);

UBLOX_transResult powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources);

UBLOX_transResult getMessageSettings(uint8_t msgClass, uint8_t msgID, uint8_t *rx_mes);

uint8_t parseNMEA(void);

UBLOX_transResult createBackup(void);

UBLOX_transResult restoreBackupData(void);

UBLOX_transResult resetReceiver(uint16_t startSelect, uint8_t start_stop);

void ublox_transmit_message(uint8_t cmd, UART_select device);

void init_gps_data(void);

void ublox_transmit_rtc(uint8_t cmd, UART_select device);

UBLOX_transResult setPowerSaveMode(uint8_t mode);

UBLOX_transResult UbloxI2CWriteReadPolling(uint16_t DevAddress, uint8_t *TData, uint16_t TDataLen,
										uint8_t *RData, uint16_t RDataLen, uint32_t Timeout);

UBLOX_transResult powerModeSetupGet(void);

UBLOX_transResult powerManageCfgGet(uint8_t *payload);

UBLOX_transResult powerManageCfgSet(uint8_t maxAckTime);


#endif /* SRC_GPS_NEOM9N_H_ */
