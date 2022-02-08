/*
 * esp.h
 *
 *  Created on: 27 џэт. 2022 у.
 *      Author: Tr
 */

#ifndef ESP_H_
#define ESP_H_

#include <stm32f0xx.h>
#include "usart.h"

#define COUNT_NUM_CMD	9
enum NUM_CMD {
	no_cmd = 0,
	cmd_reset = 1,
	cmd_set_mode,
	cmd_connect,
	cmd_disconnect,
	cmd_isConnected,
	cmd_getIP,
	cmd_multConn,
	cmd_createServer,
};

#define ESP_CMD_RESET		"AT+RST\r\n\0"
#define ESP_CMD_MODE		"AT+CWMODE=\0"
#define ESP_ANS_MODE		"AT+CWMODE=3\r\r\n\0"
#define ESP_CMD_CONNECT		"AT+CWJAP=\0"
#define ESP_CMD_isCONNECTED	"AT+CWJAP?\0"
#define ESP_ANS_CONNECT		"AT+CWJAP=\0....\r\r\n"
#define ESP_CMD_SETMULTCONN	"AT+CIPMUX=1\r\n"
#define ESP_ANS_SETMULTCONN	"AT+CIPMUX=1\r\r\n"
#define ESP_CMD_CONF_TCP_SERV	"AT+CIPSERVER=1,\0"	//need add a tcp port

#define ESP_CMD_DISCONNECT	"AT+CWQAP=\0"
#define ESP_CMD_GET_ST_CONN	"AT+CWJAP?\r\n\0"
#define ESP_CMD_GETIP		"AT+CIPSTA?\r\n\0"

#define ESP_ANS_OK			"OK\r\n"
#define ESP_ANS_FAIL		"FAIL\r\n"
#define ESP_ANS_RESET		"AT+RST\r\r\n\0"
#define ESP_ANS_DISCONNECT	"WIFI DISCONNECT\r\n"
#define ESP_ANS_GOT_IP		"WIFI GOT IP\r\n"
#define ESP_ANS_GETIP		"AT+CIPSTA?\r\r\n\0"
#define ESP_ANS_IP			"+CIPSTA:ip:\0"
#define ESP_ANS_BUSY		"busy p...\r\n\0"

#define ESP_MODE1	"1\r\n\0"
#define ESP_MODE2	"2\r\n\0"
#define ESP_MODE3	"3\r\n\0"

#define	LED_STATUS_ON	0
#define	LED_STATUS_OFF	1
#define ESP_LED(STATUS)	(GPIOC->BSRR = 1 << (8 + 16*(STATUS) ))
#define ESP_LED_TOGGLE() (GPIOC->ODR ^= GPIO_ODR_8)
#define ESP_ERR_LED() (TIM17->CR1 |= TIM_CR1_CEN)
#define ESP_OK_LED() (TIM17->CR1 &= ~TIM_CR1_CEN)

#define ESP_STATUS_CONNECTED	(1 << 0)
#define ESP_STATUS_GOTIP		(1 << 1)	//cifsr
#define ESP_STATUS_BUSY			(1 << 2)

#define ESP_ERROR_CMD_OK	0
#define ESP_ERROR_CMD_BAD	1
#define ESP_ERROR_CMD_BUSY	2


void esp_init();
void esp_indication();
void esp_timerIndication();	//TIM7
void esp_reset();
uint8_t checkStatusCmd(uint8_t numCmd);
uint8_t esp_connect(const char* ssid_name, const char* pass_ssid);
uint8_t esp_isConnected();
uint8_t esp_SetWiFiMode(const char* mode);
uint8_t esp_getStatus();
uint8_t esp_getIp();
uint8_t esp_setMultConn();
uint8_t esp_createServer(uint16_t port);



/*	doit = 1 clear buffer RX
 *  doit = 2
 */
#define PD_DOIT_NORMAL	0
#define PD_DOIT_CLR_RX	1
#define PD_DOIT_RESTART	2
uint8_t esp_processData(uint8_t doit);

void (*cmd_process[16])(uint8_t *data);


#endif /* ESP_H_ */
