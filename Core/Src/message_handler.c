/*
 * bus.c
 *
 *  Created on: Feb 25, 2023
 *      Author: kalai
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "gps_neoM9N.h"
#include "message_handler.h"
#include "uart.h"
#include "main.h"

static uint8_t TOKEN = 0;
static void calcChecksum(void);
static uint8_t calcDataSize(uint8_t *data, uint8_t len);
static void init_message_t(void);
static void handler(UART_select device);

extern osEventFlagsId_t wait_for_ack;


extern osThreadId_t gyroCalibrationTaskHandle;

uint8_t flag_connected_toIris = 0;

static uint8_t message_d[25]={0};
static message_t msg;

void tick_Handler(uint8_t *data){
	uint8_t mess_len = 0;
    init_message_t();

	if (!parseMessage(data, UART_NYX)){
		handler(UART_NYX);
	}
}

uint8_t send_heartbeat(UART_select device){
    uint8_t data[1] = {0};
    return transmitMessage(data, 1, SBP_CMD_HEARTBEAT, device);
}

uint8_t sendNack(UART_select device){
    if ((TOKEN == ESC) || (TOKEN == ETX) || (TOKEN == STX)){
        uint8_t msg[3] = {NACK,ESC,TOKEN};
        return uart_write(msg, 3, device, 5);
    }
    else{
        uint8_t msg[2] = {NACK,TOKEN};
        return uart_write(msg, 2, device, 5);
    }
}

uint8_t sendAck(UART_select device){
    if ((TOKEN == ESC) || (TOKEN == ETX) || (TOKEN == STX)){
        uint8_t msg[3] = {ACK,ESC,TOKEN};
        return uart_write(msg, 3, device, 5);
    }
    else{
        uint8_t msg[2] = {ACK,TOKEN};
        return uart_write(msg, 2, device, 5);
    }
}

static uint8_t calcDataSize(uint8_t *data, uint8_t len){
    uint8_t j,i;
    j=0;
    for (i=0;i<len;i++){
       if ((data[i] == STX) || (data[i] == ETX) || (data[i] == ESC)){
           j++;
           j++;
       }
       else j++;
    }
    return j;
}

uint8_t transmitMessage(uint8_t *data, uint8_t data_len, uint8_t cmd, UART_select device){
    uint8_t message[34];
    uint8_t i,j,index;
    uint8_t tmp_len = 0;

    tmp_len = calcDataSize(data, data_len);
    msg.len = data_len;
    memcpy(msg.data, data, msg.len);
    msg.protocol_rev[0] = PROTOCOL_VER;
    msg.protocol_rev[1] = COMM_PROTOCOL_REV;
    if (TOKEN == 255){
        TOKEN=0;
    }
    else{
        TOKEN++;
    }
    msg.token = TOKEN;

    msg.cmd = cmd;
    msg.senderID[0] = ESC;
    msg.senderID[1] = SBP_S_ID;
    calcChecksum();
    //CREATE MESSAGE
    index = 0;
    message[index] = STX;
    index++;
    message[index] = msg.protocol_rev[0];
    index++;
    message[index] = msg.protocol_rev[1];
    index++;
    if ((msg.token == STX) || (msg.token == ETX) || (msg.token == ESC)){
        message[index] = ESC;
        index++;
        message[index] = msg.token;
        index++;
    }
    else{
        message[index] = msg.token;
        index++;
    }
    message[index] = msg.senderID[0];
    index++;
    message[index] = msg.senderID[1];
    index++;
    message[index] = msg.cmd;
    index++;
    for(i=0; i<msg.len;i++){
        if ((msg.data[i] == STX) || (msg.data[i] == ETX) || (msg.data[i] == ESC)){
            message[index] = ESC;
            index++;
            message[index] = msg.data[i];
            index++;
        }
        else{
            message[index] = msg.data[i];
            index++;
        }
    }
    message[index] = msg.checksum[3];
    index++;
    message[index] = msg.checksum[2];
    index++;
    message[index] = msg.checksum[1];
    index++;
    if (msg.checksum[0] == ESC){
        message[index] = ESC;
        index++;
        message[index] = msg.checksum[0];
        index++;
        message[index] = ETX;
        index++;
        msg.len = index;
    }
    else{
        message[index] = msg.checksum[0];
        index++;
        message[index] = ETX;
        index++;
        msg.len = index;
    }

    uart_write(message, msg.len, device, 10);
    osEventFlagsSet(wait_for_ack, 0x00000001U);
    return 1;
}

static void calcChecksum(void){
    msg.checksum[0] = msg.protocol_rev[0];
    msg.checksum[0] ^= msg.protocol_rev[1];

    msg.checksum[0] ^= msg.token;

    msg.checksum[0] ^= msg.senderID[0];
    msg.checksum[0] ^= msg.senderID[1];

    msg.checksum[0] ^= msg.cmd;
    uint8_t i;

    for (i=0; i < msg.len; i++)
    {
       msg.checksum[0] ^= msg.data[i];
    }
}


uint8_t parseMessage(uint8_t *data, UART_select device){
    uint8_t chsum = 0;
    uint8_t len = data[0];
    if (len < 11){
        sendNack(device);
        return 1;
    }
    msg.len = len-10;
    if (!msg.data)return 2;
    msg.protocol_rev[0] = data[1];
    msg.protocol_rev[1] = data[2];
    msg.token = data[3];
    TOKEN = msg.token;
    msg.senderID[0] = data[4];
    msg.senderID[1] = data[5];
    msg.cmd = data[6];
    uint8_t i;
    for (i=0; i<msg.len; i++){
//        if (data[6+i] == ESC) i++;
        msg.data[i] = data[7+i];
    }
    msg.checksum[3] = data[7+i];
    msg.checksum[2] = data[8+i];
    msg.checksum[1] = data[9+i];
    msg.checksum[0] = data[10+i];
    chsum = msg.checksum[0];
    calcChecksum();
    if (chsum != msg.checksum[0]){
        sendNack(device);
        return 1;
    }
    sendAck(device);
//    HAL_Delay(22);
    return 0;                  //Note that after the parsing the escape chars remains in payload.
}



void handler(UART_select device){
    switch (msg.cmd){
    case 0x50:
        flag_connected_toIris = 1;
        break;
    case 0x51:break;
    case 0x56:
        ublox_transmit_message(msg.cmd, device);
        break;
    case 0x65:
        reportFW(msg.cmd, device);
        break;
    case 0x78:
    	powerManageCfgSet(0x78);
        break;
    case 0x80:
        ublox_transmit_rtc(msg.cmd, device);
        break;
    case 0xA0:
//        gpio_setGNSS_RESET(PIN_LOW);
        HAL_Delay(500);
//        gpio_setGNSS_RESET(PIN_HIGH);
        break;
    case 0xC0:
    	osThreadResume(gyroCalibrationTaskHandle);
    	break;
    case 0xC1:
    	magnCalStart();
		break;
    case 0xC2:
		accCalStart();
		break;
    default:
        break;
    }
}

void init_message_t(void){
    msg.protocol_rev[0] = 0;
    msg.protocol_rev[1] = 0;
    msg.token = 0;
    msg.senderID[0] = 0;
    msg.senderID[1] = 0;
    msg.cmd = 0;
    memset(msg.data, 0, 20);
    msg.checksum[0] = 0;
    msg.checksum[1] = 0;
    msg.checksum[2] = 0;
    msg.checksum[3] = 0;
}

void reportFW(uint8_t cmd, UART_select device){
    uint8_t fwv[1] = {FW_VERSION};
    transmitMessage(fwv, 1, cmd, device);
}





