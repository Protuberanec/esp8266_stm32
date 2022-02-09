/*
 * esp.c
 *
 *  Created on: 27 џэт. 2022 у.
 *      Author: Tr
 */

#include "esp.h"

uint8_t curCMD = 0;
uint8_t cmd_ok[COUNT_NUM_CMD] = {0};
uint8_t esp_status = 0;
uint8_t connectedUser = 0;
uint8_t currentIP[16];

uint8_t currentData = 0;
struct DataFromClient clientData[2];

void TIM17_IRQHandler() {
	TIM17->SR &= ~TIM_SR_UIF;
	ESP_LED_TOGGLE();
}



uint8_t checkStatusCmd(uint8_t numCmd) {
	uint16_t temp = 0;
	while(!cmd_ok[curCMD]) {
		temp += 1 - esp_processData(PD_DOIT_NORMAL);
		if ((esp_getStatus() & ESP_STATUS_BUSY) == ESP_STATUS_BUSY) {
			for (int i = 0; i < 50000; i++);
			return ESP_ERROR_CMD_BAD;
		}
		if (temp > 20000) {
			return ESP_ERROR_CMD_BAD;	//answer is not coming
		}
	}

	return ESP_ERROR_CMD_OK;
}

void esp_init() {
	esp_indication();
	esp_timerIndication();
	USART2_Init();
}

void esp_indication() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
}

void esp_timerIndication() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	TIM17->ARR = SystemCoreClock / 1000;
	TIM17->PSC = SystemCoreClock / (SystemCoreClock / 1000);
	TIM17->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM17_IRQn, 15);
	NVIC_EnableIRQ(TIM17_IRQn);

//	TIM17->CR1 |= TIM_CR1_CEN;
}

uint8_t esp_getStatus() {
	return esp_status;
}

uint8_t esp_DataProcessed() {
	esp_status &= ~ESP_STATUS_NEW_DATA0;
}

void esp_reset() {
	curCMD = cmd_reset;
	esp_processData(PD_DOIT_CLR_RX);	//clear buffer....
	USART_PutString(ESP_CMD_RESET, sizeStr(ESP_CMD_RESET));
	USART_SendDataFromBuffer();

	for (int i = 0; i < 50000; i++);
	while (esp_processData(PD_DOIT_NORMAL));

	if (!cmd_ok[cmd_reset]) {
		int a = 10;
	}
}

uint8_t esp_connect(const uint8_t* ssid_name, const uint8_t* ssid_pass) {
	curCMD = cmd_connect;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_CONNECT, sizeStr(ESP_CMD_CONNECT));
	USART_PutString("\"", 1);
	USART_PutString(ssid_name, sizeStr(ssid_name));
	USART_PutString("\",\"", 3);
	USART_PutString(ssid_pass, sizeStr(ssid_pass));
	USART_PutString("\"\r\n", 3);
	USART_SendDataFromBuffer();

	for (int i = 0; i < 150000; i++);	//wait while not connected....

	return checkStatusCmd(curCMD);
}

uint8_t esp_SetWiFiMode(const uint8_t* mode) {
	curCMD = cmd_set_mode;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_MODE, sizeStr(ESP_CMD_MODE));
	USART_PutString(mode, sizeStr(mode));
	USART_SendDataFromBuffer();

	return checkStatusCmd(curCMD);
}

uint8_t esp_isConnected() {
	curCMD = cmd_isConnected;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_isCONNECTED, sizeStr(ESP_CMD_isCONNECTED));
	USART_SendDataFromBuffer();

	uint16_t counter = 0;
	while(!cmd_ok[curCMD] && counter++ < 10000) {
		esp_processData(PD_DOIT_NORMAL);
	}

	if (esp_processData(PD_DOIT_NORMAL) == 0) {

	}
	return 0;
}

uint8_t esp_getIp() {
	curCMD = cmd_getIP;
	esp_status &= ~ESP_STATUS_BUSY;

	USART_PutString(ESP_CMD_GETIP, sizeStr(ESP_CMD_GETIP));
	USART_SendDataFromBuffer();

	return checkStatusCmd(curCMD);
}

uint8_t esp_setMultConn() {
	curCMD = cmd_multConn;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_CIPMUX, sizeStr(ESP_CMD_CIPMUX));
	USART_SendDataFromBuffer();

	return checkStatusCmd(curCMD);
}


uint8_t esp_createServer(uint16_t port) {
	while (esp_setMultConn() != ESP_ERROR_CMD_OK);

	char strPort[5] = {0};
	intToStr(port, &strPort[0]);	//very specific function!!!

	curCMD = cmd_createServer;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_CONF_TCP_SERV, sizeStr(ESP_CMD_CONF_TCP_SERV));
//	USART_PutString(strPort, 5);
//	USART_PutString("\r\n",2);

	USART_SendDataFromBuffer();
	return checkStatusCmd(curCMD);
}

uint8_t esp_setTimeout(uint16_t sec) {
	curCMD = cmd_cipsto;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_CIPSTO, sizeStr(ESP_CMD_CIPSTO));
	USART_SendDataFromBuffer();

	return checkStatusCmd(curCMD);
}

uint8_t esp_sendToClinetData(uint8_t id, const uint8_t* data, uint8_t size) {
	curCMD = cmd_sendex;
	esp_status &= ~ESP_STATUS_BUSY;
	if (!(connectedUser & (1 << id))) {
		return ESP_ERROR_NO_CLIENT;
	}

//	ESP_CMD_CIPSENDEX
	//AT+CIPSENDEX=id,len
	//

}

uint8_t esp_processData(uint8_t doit) {
	uint8_t espByte;

	if (ProcessBuffer(&espByte) == 0) {
		return 0;
	}

	static uint8_t ptrCmd = 0;
	static uint8_t cmd[64] = {0};
	static uint8_t ptrData = 0;
	static uint8_t sizeData = 0;
	static uint8_t countCR = 0;	//var for count \r, when answer on cmd it's must be 2
	static uint8_t statusData = 0;

	if (doit == PD_DOIT_RESTART) {
		ptrCmd = 0;
		countCR = 0;
	}

	if (doit == PD_DOIT_CLR_RX) {
		clearRxBuffer();
		countCR = 0;
		ptrCmd = 0;
		return 0;
	}

	if (statusData) {
		switch (statusData) {
			case 1 :	//get ID client
				if (espByte == ',') {
					++statusData;
					ptrData = 0;
					break;
				}
				clientData[0].id_client = espByte - 0x30;
			break;
			case 2 : //get size data
				if (espByte == ':') {
					++statusData;
					clientData[0].size = strToInt(&clientData[0].data[0], ptrData);
					sizeData = clientData[0].size;
					ptrData = 0;
					break;
				}
				clientData[0].data[ptrData++] = espByte;

			break;
			case 3 : // get data
				if ((clientData[0].size - ptrData) == 1) {
					//get last byte, and message about new accepted data!!!
					statusData = 0;	//end accepted data...
					esp_status |= ESP_STATUS_NEW_DATA0;
				}
				clientData[0].data[ptrData++] = espByte;
				clientData[0].data[ptrData] = 0x00;

			break;

		}
		return 1;
	}

	cmd[ptrCmd++] = espByte;
	cmd[ptrCmd] = 0x00;



	if (espByte == '\r') {
		++countCR;
		statusData = 0;
	}
	else if (strCmp(ESP_ANS_NEW_DATA, cmd) == 0) {	//wait new data came...
		++ptrData;
		statusData = 1;
	}
	else if (espByte == '\n' && countCR > 0) {
		if (strCmp(ESP_ANS_RESET, cmd) == 0) {
			cmd_ok[curCMD] = 0;
			ESP_OK_LED();
			ESP_LED(LED_STATUS_OFF);
			//timer is off....
		}
		else if (strCmp(ESP_ANS_OK, cmd) == 0) {
			cmd_ok[curCMD] = 1;
			ESP_OK_LED();
			ESP_LED(LED_STATUS_ON);
		}
		else if (strCmp(ESP_ANS_FAIL, cmd) == 0) {
			ESP_ERR_LED();
		}
		else if (strCmp(ESP_ANS_ERROR, cmd) == 0) {
			ESP_ERR_LED();
		}
		else if (strCmp(ESP_ANS_DISCONNECT, cmd) == 0) {
			ESP_ERR_LED();
			esp_status &= ~ESP_STATUS_CONNECTED;
		}
		else if (strCmp(ESP_ANS_GOT_IP, cmd) == 0) {
			cmd_ok[cmd_connect] = 1;
			cmd_ok[cmd_isConnected] = 1;
			ESP_OK_LED();
			ESP_LED(LED_STATUS_ON);
			esp_status |= ESP_STATUS_CONNECTED;
		}
		else if (strCmp(ESP_ANS_GETIP, cmd) == 0) {
			ESP_OK_LED();
			ESP_LED(LED_STATUS_ON);
		}
		else if (Str1inStr2(ESP_ANS_IP, cmd) == 0) {
			//I have found IP
			esp_status |= ESP_STATUS_GOTIP;
			uint8_t sizeIp = sizeStr(cmd) - 11 - 2 /*""*/ - 2 /*\r\n*/;
			strnCpy(cmd, currentIP, 12, sizeIp);
		}
		else if (strCmp(ESP_ANS_BUSY, cmd) == 0) {
			esp_status |= ESP_STATUS_BUSY;
		}
		else if (strCmp(ESP_ANS_CIPMUX, cmd) == 0) {
			cmd_ok[cmd_multConn] = 0;
		}
		else if (cmd[0] >= '0' && cmd[0] <= '4') {
			// it's may be connected new user... or disconnected
			if (strCmp(ESP_ANS_USER_CONNECTED, &cmd[2]) == 0) {
				//new user connected
				connectedUser |= 1 << (cmd[0] - 0x30);
				ESP_LED_USER_CONN();
			}
			if (strCmp(ESP_ANS_USER_DISCONNECTED, &cmd[2]) == 0) {
				//user disconnected
				connectedUser &= ~(1 << (cmd[0] - 0x30));
				ESP_LED_USER_DISCON();
			}
		}

		ptrCmd = 0;
		countCR = 0;
	}

	if (ptrCmd > 64) {
		ptrCmd = 0;
	}
//	cmd[ptrCmd++] = espByte;


	return 1;

}


