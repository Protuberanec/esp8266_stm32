/*
 * esp.h
 *
 *  Created on: 27 џэт. 2022 у.
 *      Author: Tr
 */

#ifndef ESP_H_
#define ESP_H_

#include <stm32f0xx.h>
#include <math.h>
#include "usart.h"
#include "my_str.h"

#define COUNT_NUM_CMD	12
enum NUM_CMD {
	no_cmd = 0,
	cmd_reset = 1,
	cmd_set_mode,
	cmd_connect,
	cmd_disconnect,
	cmd_isConnected,
	cmd_getIP,	//6
	cmd_multConn,
	cmd_createServer,
	cmd_cipsto,
	cmd_sendex,
	cmd_data_send_ok,	//11
};

#define ESP_CMD_RESET		((uint8_t*)"AT+RST\r\n\0")
#define ESP_CMD_MODE		((uint8_t*)"AT+CWMODE=\0")
#define ESP_ANS_MODE		((uint8_t*)"AT+CWMODE=3\r\r\n\0")
#define ESP_CMD_CONNECT		((uint8_t*)"AT+CWJAP=\0")
#define ESP_CMD_isCONNECTED	((uint8_t*)"AT+CWJAP?\0")
#define ESP_ANS_CONNECT		((uint8_t*)"AT+CWJAP=\0....\r\r\n\0")
#define ESP_CMD_CIPMUX		((uint8_t*)"AT+CIPMUX=1\r\n\0")
#define ESP_ANS_CIPMUX		((uint8_t*)"AT+CIPMUX=1\r\r\n\0")
#define ESP_CMD_CONF_TCP_SERV	((uint8_t*)"AT+CIPSERVER=1,30000\r\n\0")	//need add a tcp port
#define ESP_CMD_CIPSTO		((uint8_t*)"AT+CIPSTO=1800\r\n\0")
#define ESP_ANS_CIPSTO		((uint8_t*)"AT+CIPSTO=1800\r\r\n\0")
#define ESP_CMD_CIPSENDEX	((uint8_t*)"AT+CIPSENDEX=\0")
#define ESP_ANS_CIPSENDEX	((uint8_t*)"AT+CIPSENDEX=\0")	//need add id and len
#define ESP_ANS_SEND_OK		((uint8_t*)"SEND OK\r\n\0")	//when data is fine going out...


#define ESP_CMD_DISCONNECT	((uint8_t*)"AT+CWQAP=\0")
#define ESP_CMD_GET_ST_CONN	((uint8_t*)"AT+CWJAP?\r\n\0")
#define ESP_CMD_GETIP		((uint8_t*)"AT+CIPSTA?\r\n\0")

#define ESP_ANS_OK			((uint8_t*)"OK\r\n")
#define ESP_ANS_FAIL		((uint8_t*)"FAIL\r\n")
#define ESP_ANS_ERROR		((uint8_t*)"ERROR\r\n")
#define ESP_ANS_RESET		((uint8_t*)"AT+RST\r\r\n\0")
#define ESP_ANS_DISCONNECT	((uint8_t*)"WIFI DISCONNECT\r\n")
#define ESP_ANS_GOT_IP		((uint8_t*)"WIFI GOT IP\r\n")
#define ESP_ANS_GETIP		((uint8_t*)"AT+CIPSTA?\r\r\n\0")
#define ESP_ANS_IP			((uint8_t*)"+CIPSTA:ip:\0")
#define ESP_ANS_BUSY		((uint8_t*)"busy p...\r\n\0")
#define ESP_ANS_BUSY_SEND	((uint8_t*)"busy s...\r\n\0")	//while send data
#define ESP_ANS_USER_CONNECTED	((uint8_t*)"CONNECT\r\n")
#define ESP_ANS_USER_DISCONNECTED	((uint8_t*)"CLOSED\r\n")
#define ESP_ANS_NEW_DATA	((uint8_t*)"+IPD,")

#define ESP_MODE1	((uint8_t*)"1\r\n\0")
#define ESP_MODE2	((uint8_t*)"2\r\n\0")
#define ESP_MODE3	((uint8_t*)"3\r\n\0")

#define	LED_STATUS_ON	0
#define	LED_STATUS_OFF	1
#define ESP_LED(STATUS)	(GPIOC->BSRR = 1 << (8 + 16*(STATUS) ))
#define ESP_LED_TOGGLE() (GPIOC->ODR ^= GPIO_ODR_8)
#define ESP_LED_USER_CONN() (GPIOC->ODR |= GPIO_ODR_9)
#define ESP_LED_USER_DISCON() (GPIOC->ODR &= ~GPIO_ODR_9)
#define ESP_ERR_LED() (TIM17->CR1 |= TIM_CR1_CEN)
#define ESP_OK_LED() (TIM17->CR1 &= ~TIM_CR1_CEN)

#define ESP_STATUS_CONNECTED	(1 << 0)
#define ESP_STATUS_GOTIP		(1 << 1)	//cifsr
#define ESP_STATUS_BUSY			(1 << 2)
#define ESP_STATUS_NEW_DATA0	(1 << 3)
#define ESP_STATUS_NEW_DATA2	(1 << 4)

#define ESP_ERROR_CMD_OK	0
#define ESP_ERROR_CMD_BAD	1
#define ESP_ERROR_CMD_BUSY	2
#define ESP_ERROR_NO_CLIENT	3

#define ESP_DATA_SEND_OK	1
#define ESP_DATA_SEND_NO	0

struct DataFromClient {
	uint8_t id_client;
	uint8_t data[64];
	uint8_t size;
};


void esp_init();
void esp_indication();
void esp_timerIndication();	//TIM7
void esp_reset();
uint8_t checkStatusCmd(uint8_t numCmd);
uint8_t esp_connect(const uint8_t* ssid_name, const uint8_t* pass_ssid);
uint8_t esp_isConnected();
uint8_t esp_SetWiFiMode(const uint8_t* mode);
uint8_t esp_getStatus();
uint8_t esp_DataProcessed();
uint8_t esp_getIp();
uint8_t esp_setMultConn();
uint8_t esp_createServer(uint16_t port);
uint8_t esp_setTimeout(uint16_t sec);

uint8_t esp_sendToClinetData(uint8_t id, const uint8_t* data, uint16_t size);
uint8_t esp_isSendData();



/*	doit = 1 clear buffer RX
 *  doit = 2
 */
#define PD_DOIT_NORMAL	0
#define PD_DOIT_CLR_RX	1
#define PD_DOIT_RESTART	2
uint8_t esp_processData(uint8_t doit);

void (*cmd_process[16])(uint8_t *data);


#endif /* ESP_H_ */
