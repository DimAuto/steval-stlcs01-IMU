/*
 * gps_neoM9N.c
 *
 *  Created on: Feb 7, 2023
 *      Author: dkalaitzakis
 */

#include "gps_neoM9N.h"
#include <stdio.h>
#include <string.h>


static void calcChecksum(messageCFG_t *msg);

// I2C object
I2C_HandleTypeDef hi2c1;

//gps_data_t *gps_data;
static uint8_t powerModesetPld[44] = {0x01, 0x06, 0x78, 0x00, 0x0E, 0x10, 0x42, 0x01, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00,
		0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00};
static messageCFG_t config_message = {0, 0, 0, 0, 0, 0, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
static gps_data_t gps_data;
static uint8_t gps_loss_count = 0;

uint8_t gps_data_backup_flag = 0;   //Flag to enable gps data backup only on boot.


uint8_t ublox_i2c_bus_init(void){
	hi2c1.Instance = I2C1;
//	hi2c1.Init.Timing = 0x00B03FDB; 400KB i2c speed
	hi2c1.Init.Timing = 0x307075B1;	//100KB i2c speed
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
	return 1;
	}

	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
	return 2;
	}

	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
	return 3;
	}
	return 0;
}


void ublox_tick(void){
    uint8_t res = 0;
    res = ubloxRead();
    if ((res == 8) || (res==10)){
#ifdef __DEBUG__
        uart_write_debug("Failed to read\r\n",UART_NYX);
#endif
        return;
    }
    else{
        parseNMEA();
    }
}

void ublox_transmit_rtc(uint8_t cmd, UART_select device){
    transmitMessage(gps_data.timestamp, 9, cmd, device);
}

void ublox_transmit_message(uint8_t cmd, UART_select device){
    uint8_t message[12] = {0};
    message[0] = (gps_data.latitude & 0xFF000000) >> 24;
    message[1] = (gps_data.latitude & 0x00FF0000) >> 16;
    message[2] = (gps_data.latitude & 0x0000FF00) >> 8;
    message[3] = (gps_data.latitude & 0x000000FF);
    message[4] = (gps_data.longtitude & 0xFF000000) >> 24;
    message[5] = (gps_data.longtitude & 0x00FF0000) >> 16;
    message[6] = (gps_data.longtitude & 0x0000FF00) >> 8;
    message[7] = (gps_data.longtitude & 0x000000FF);
    message[8] = (gps_data.altitude & 0xFF000000) >> 24;
    message[9] = (gps_data.altitude & 0x00FF0000) >> 16;
    message[10] = (gps_data.altitude & 0x0000FF00) >> 8;
    message[11] = (gps_data.altitude & 0x000000FF);
    transmitMessage(message, 12, cmd, device);
}

UBLOX_transResult ubloxInit(void){
	UBLOX_transResult ret;
    ret = setPortOutput(COM_PORT_I2C, COM_TYPE_NMEA);
    if (ret != UBX_ACK){
		uart_write_debug(" POUT:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GLL, NMEA_GGL_RATE, COM_PORT_I2C);
    if (ret != UBX_ACK){
    	uart_write_debug(" GGL:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GSA, NMEA_GSA_RATE, COM_PORT_I2C);
    if (ret != UBX_ACK){
		uart_write_debug(" GSA:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GSV, NMEA_GSV_RATE, COM_PORT_I2C);
    if (ret != UBX_ACK){
		uart_write_debug(" GSV:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_RMC, NMEA_RMC_RATE, COM_PORT_I2C);
    if (ret != UBX_ACK){
		uart_write_debug(" RMC:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_VTG, NMEA_VTG_RATE, COM_PORT_I2C);
    if (ret != UBX_ACK){
		uart_write_debug(" VTG:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GGA, NMEA_GGA_RATE, COM_PORT_I2C);
    if (ret != UBX_ACK){
		uart_write_debug(" GGA:%d\r\n", 50);
    }
    HAL_Delay(10);
    ret = powerManageCfgSet(120);
    if (ret != UBX_ACK){
		uart_write_debug(" PM2:%d\r\n", 50);
    }
	return ret;
}

UBLOX_transResult ubloxNmeaGGA_set_refresh_rate(uint8_t seconds){
    return configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GGA, seconds, COM_PORT_I2C);
}

uint8_t ubloxRead(void){
    uint8_t res = 0;

    uint16_t num = 0;
    uint8_t bytes[2] = {0};

    res = HAL_I2C_Mem_Read(&hi2c1, UBLOX_M9N, 0xFD, I2C_MEMADD_SIZE_8BIT, bytes, 2, 20);
    if (res!=HAL_OK)return res;
    num  = ((bytes[0] << 8) | bytes[1]);
    memset(bytes, 0, 2);
    if (num > 0){
    	if (num>75)num=75;
        res = HAL_I2C_Mem_Read(&hi2c1, UBLOX_M9N, 0xFF, I2C_MEMADD_SIZE_8BIT, gps_data.sentence, num, 100);
        if ((res != HAL_OK) || (gps_data.sentence[0] != '$')){
                return 10;
        }
#ifdef __DEBUG__
        uart_write_debug(gps_data.sentence, 50);
        uart_write_debug("\r\n", 10);
#endif
        return res;
    }
    return 10;
}

uint8_t parseNMEA(void){
    char lat[12] = {0};
    char lng[12] = {0};
    char alt[7] = {0};
    const char NMEA_delimiter[2] = ",";
    char * token = strtoke(gps_data.sentence, NMEA_delimiter);

    uint8_t i = 0;
    for (i = 0; token != NULL; i++) {
        switch (i) {
        case 0:
            break;
        case 1:
            strcpy(gps_data.timestamp, token);
            break;
        case 2:
            strcpy(lat, token);
            break;
        case 3:
            strcpy(gps_data.NS, token);
            break;
        case 4:
            strcpy(lng, token);
            break;
        case 5:
            strcpy(gps_data.EW, token);
            break;
        case 6:
            strcpy(gps_data.quality, token);
            break;
        case 7:
            strcpy(gps_data.satellites, token);
            break;
        case 8:
            strcpy(gps_data.HDOP, token);
            break;
        case 9:
            strcpy(alt, token);
            break;
        case 11:
            strcpy(gps_data.sep, token);
            break;
        }
        token = strtoke(NULL, NMEA_delimiter);
    }
    if (i<11){ //If the number of fields parsed is less than 11. Return error.
        gps_loss_count++;
        if (gps_loss_count > GPS_LOSS_COUNT_THR){
            init_gps_data();
        }
        return 1;
    }
    if ((gps_data.quality[0] == '1') || (gps_data.quality[0] == '2') || (gps_data.quality[0] == '4') || (gps_data.quality[0] == '5')){
        gps_data.latitude = coorsAtol(lat, gps_data.NS);
        gps_data.longtitude = coorsAtol(lng, gps_data.EW);
        gps_data.altitude = altAtol(alt);
        gps_loss_count = 0;
    }
    else if (gps_data.quality[0] == '0'){
        gps_loss_count++;
        if (gps_loss_count > GPS_LOSS_COUNT_THR){
            init_gps_data();
        }
    }
    memset(gps_data.sentence, 0, 75);
    return 0;
}


static void calcChecksum(messageCFG_t *msg){
    msg->checksumA = 0;
    msg->checksumB = 0;

    msg->checksumA += msg->cls;
    msg->checksumB += msg->checksumA;

    msg->checksumA += msg->id;
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len & 0xFF);
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len >> 8);
    msg->checksumB += msg->checksumA;

    uint8_t i;
    for (i=0; i < msg->len; i++)
    {
        msg->checksumA += msg->payload[i];
        msg->checksumB += msg->checksumA;
    }
}

UBLOX_transResult sendI2Cmessage(void){
	UBLOX_transResult res;
    uint8_t message[60] = {0};
    uint8_t rx_message[20] = {0};
    uint8_t len = config_message.len + 8;
    message[0] = UBX_SYNCH_1;
    message[1] = UBX_SYNCH_2;
    message[2] = config_message.cls;
    message[3] = config_message.id;
    message[4] = (config_message.len & 0xFF);
    message[5] = (config_message.len >> 8);
    uint8_t i;
    for ( i=0 ; i < config_message.len ; i++){
        message[6+i] = config_message.payload[i];
    }
    message[6+i] = config_message.checksumA;
    message[7+i] = config_message.checksumB;
    res = UbloxI2CWriteReadPolling(UBLOX_M9N, message, len, rx_message, 20, 50);
    if (res == TRANS_OK){
    	for(i=0; i<20;i++){
    		if(rx_message[i] == UBX_SYNCH_1){
    			break;
    		}
    	}
    	return rx_message[i+3];  //UBLOX returns 1 for ACK and 0 for NACK
    }
    return res;
}

UBLOX_transResult setPortOutput(uint8_t portSelect, uint8_t streamSettings){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_PRT;
    config_message.len = 20;
    uint8_t payloadCfg[20] = {0};
    payloadCfg[4] = 0x84;
    payloadCfg[12] = 0x23;
    payloadCfg[14] = streamSettings;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult ubloxSaveConfiguration(void){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_CFG;
    config_message.len = 12;
    uint8_t payloadCfg[12] = {0};
    payloadCfg[4] = 0xFF;
    payloadCfg[5] = 0xFF;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult ubloxLoadConfiguration(void){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_CFG;
    config_message.len = 12;
    uint8_t payloadCfg[12] = {0};
    payloadCfg[8] = 0xFF;
    payloadCfg[9] = 0xFF;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult ubloxResetConfiguration(void){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_CFG;
    config_message.len = 12;
    uint8_t payloadCfg[12] = {0};
    payloadCfg[0] = 0xFF;
    payloadCfg[1] = 0xFF;
    payloadCfg[8] = 0xFF;
    payloadCfg[9] = 0xFF;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult getPortSettings(uint8_t portID, uint8_t *rx_mes){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_PRT;
    config_message.len = 1;
    uint8_t payloadCfg[1] = {0};
    payloadCfg[0] = portID;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    uint8_t message[10] = {0};
    message[0] = UBX_SYNCH_1;
    message[1] = UBX_SYNCH_2;
    message[2] = config_message.cls;
    message[3] = config_message.id;
    message[4] = (config_message.len & 0xFF);
    message[5] = (config_message.len >> 8);
    message[6] = payloadCfg[0];
    message[7] = config_message.checksumA;
    message[8] = config_message.checksumB;
    return UbloxI2CWriteReadPolling(UBLOX_M9N, message, 10, rx_mes, 11, 100);
}

UBLOX_transResult getMessageSettings(uint8_t msgClass, uint8_t msgID, uint8_t *rx_mes){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_MSG;
    config_message.len = 2;
    uint8_t payloadCfg[2] = {0};
    payloadCfg[0] = msgClass;
    payloadCfg[1] = msgID;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    uint8_t message[10] = {0};
    message[0] = UBX_SYNCH_1;
    message[1] = UBX_SYNCH_2;
    message[2] = config_message.cls;
    message[3] = config_message.id;
    message[4] = (config_message.len & 0xFF);
    message[5] = (config_message.len >> 8);
    message[6] = payloadCfg[0];
    message[7] = payloadCfg[1];
    message[8] = config_message.checksumA;
    message[9] = config_message.checksumB;
    return UbloxI2CWriteReadPolling(UBLOX_M9N, message, 10, rx_mes, 11, 100);
}

UBLOX_transResult configureNMEA(uint8_t msgClass, uint8_t msgID, uint8_t rate, uint8_t portID){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_MSG;
    config_message.len = 8;
    uint8_t payloadCfg[8] = {0};
    payloadCfg[0] = msgClass;
    payloadCfg[1] = msgID;
    payloadCfg[2] = rate;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources){
    config_message.cls = UBX_CLASS_RXM;
    config_message.id = UBX_RXM_PMREQ;
    config_message.len = 16;
    uint8_t payloadCfg[16] = {0};
    payloadCfg[0] = 0x00; // message version
    // bytes 1-3 are reserved - and must be set to zero
    payloadCfg[1] = 0x00;
    payloadCfg[2] = 0x00;
    payloadCfg[3] = 0x00;
    payloadCfg[4] = (durationInMs >> (8 * 0)) & 0xff;
    payloadCfg[5] = (durationInMs >> (8 * 1)) & 0xff;
    payloadCfg[6] = (durationInMs >> (8 * 2)) & 0xff;
    payloadCfg[7] = (durationInMs >> (8 * 3)) & 0xff;
    payloadCfg[8] = 0x06;
    payloadCfg[9] = 0x00;
    payloadCfg[10] = 0x00;
    payloadCfg[11] = 0x00;
    payloadCfg[12] = (wakeupSources >> (8 * 0)) & 0xff;
    payloadCfg[13] = (wakeupSources >> (8 * 1)) & 0xff;
    payloadCfg[14] = (wakeupSources >> (8 * 2)) & 0xff;
    payloadCfg[15] = (wakeupSources >> (8 * 3)) & 0xff;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();

}

UBLOX_transResult setPowerSaveMode(uint8_t mode){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_RXM;
    config_message.len = 2;
    uint8_t payloadCfg[2] = {0};
    payloadCfg[0] = 0;
    payloadCfg[1] = mode;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult powerModeGet(void){
    uint8_t res = 0;
    uint8_t data[10] = {0};
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_RXM;
    config_message.len = 0;
    calcChecksum(&config_message);
    uint8_t message[10] = {0};
    message[0] = UBX_SYNCH_1;
    message[1] = UBX_SYNCH_2;
    message[2] = config_message.cls;
    message[3] = config_message.id;
    message[4] = (config_message.len & 0xFF);
    message[5] = (config_message.len >> 8);
    message[6] = config_message.checksumA;
    message[7] = config_message.checksumB;
    res = UbloxI2CWriteReadPolling(UBLOX_M9N, message, 8, data, 10, 100);
    if (res != HAL_OK)return res;
    return data[7];
}


UBLOX_transResult powerModeSetupGet(void){
    uint8_t res = 0;
    uint8_t data[16] = {0};
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_PMS;
    config_message.len = 0;
    calcChecksum(&config_message);
    uint8_t message[10] = {0};
    message[0] = UBX_SYNCH_1;
    message[1] = UBX_SYNCH_2;
    message[2] = config_message.cls;
    message[3] = config_message.id;
    message[4] = (config_message.len & 0xFF);
    message[5] = (config_message.len >> 8);
    message[6] = config_message.checksumA;
    message[7] = config_message.checksumB;
    res = UbloxI2CWriteReadPolling(UBLOX_M9N, message, 8, data, 16, 100);
    if (res != HAL_OK)return res;
    return data[7];
}


UBLOX_transResult powerManageCfgGet(uint8_t *payload){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_PM2;
    config_message.len = 0;
    calcChecksum(&config_message);
    uint8_t message[10] = {0};
    message[0] = UBX_SYNCH_1;
    message[1] = UBX_SYNCH_2;
    message[2] = config_message.cls;
    message[3] = config_message.id;
    message[4] = (config_message.len & 0xFF);
    message[5] = (config_message.len >> 8);
    message[6] = config_message.checksumA;
    message[7] = config_message.checksumB;
    return UbloxI2CWriteReadPolling(UBLOX_M9N, message, 8, payload, 56, 200);
}


UBLOX_transResult powerManageCfgSet(uint8_t maxAckTime){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_PM2;
    config_message.len = 44;
    config_message.payload = powerModesetPld;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}


UBLOX_transResult createBackup(void){
    config_message.cls = UBX_CLASS_UPD;
    config_message.id = 0x14;
    config_message.len = 4;
    uint8_t payloadCfg[4] = {0};
    payloadCfg[0] = 0;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult restoreBackupData(void){
    config_message.cls = UBX_CLASS_UPD;
    config_message.id = 0x14;
    config_message.len = 0;
    uint8_t payloadCfg[1] = {0};
    payloadCfg[1] = 0;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

UBLOX_transResult resetReceiver(uint16_t startSelect, uint8_t start_stop){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_RST;
    config_message.len = 4;
    uint8_t payloadCfg[4] = {0};
    payloadCfg[0] = startSelect & 0xFF;
    payloadCfg[1] = (startSelect >> 8);
    payloadCfg[3] = start_stop;
    payloadCfg[4] = 0;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage();
}

void init_gps_data(void){
    memset(gps_data.EW, 0, 1);
    memset(gps_data.HDOP, 0, 5);
    memset(gps_data.NS, 0, 1);
    gps_data.altitude = 0;
    gps_data.latitude = 0;
    gps_data.longtitude = 0;
    memset(gps_data.quality, 0, 1);
    memset(gps_data.satellites, 0, 2);
    memset(gps_data.sentence, 0, 75);
    memset(gps_data.sep, 0, 6);
    memset(gps_data.timestamp, 0, 9);
}


UBLOX_transResult UbloxI2CWriteReadPolling(uint16_t DevAddress, uint8_t *TData, uint16_t TDataLen,
										uint8_t *RData, uint16_t RDataLen, uint32_t Timeout)
{
	HAL_StatusTypeDef ret = 0x00;

	if (HAL_I2C_Master_Transmit(&hi2c1, DevAddress, TData, TDataLen, Timeout)!= HAL_OK ){
		return TRANS_ERROR;
	}
	// Read Response
	if (HAL_I2C_Master_Receive(&hi2c1, DevAddress, RData, RDataLen, Timeout) != HAL_OK){
		return RECEIVE_ERROR;
	}
	return TRANS_OK;
}



