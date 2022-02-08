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

uint8_t currentIP[16];

void TIM17_IRQHandler() {
	TIM17->SR &= ~TIM_SR_UIF;
	ESP_LED_TOGGLE();
}

uint16_t sizeStr(const char* str) {
	uint16_t size = 0;
	while(str[size++] != '\0');
	return size-1;
}

uint8_t strCmp(char* str1, char* str2) {
    int i = 0;
    while(str1[i] == str2[i]) {
        if (str1[i] == 0x00 && str2[i] == 0x00) {
            return 0;
        }
        ++i;
    }

    if (str1[i] == 0x00)
        return 2;
    return 1;
}

void strnCpy(char* source, char* dst, uint8_t startPos, uint8_t size) {
	for (int i = 0; i < size; i++) {
		dst[i] = source[startPos + i];
	}
}

uint8_t Str1inStr2(char* str1, char* str2) {
    int i = 0;
    while(str1[i] == str2[i]) {
        if (str1[i] == 0x00) {
            return 0;
        }
        if (str2[i] == 0) {
        	return 2;
        }
        ++i;
    }

    if (str1[i] == 0x00)
        return 0;
    return 1;
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

uint8_t intToStr(uint16_t num, char* numStr) {
	numStr[0] = num / 10000 + 0x30;
	num -= (num/10000) * 10000;

	numStr[1] = num/1000 + 0x30;
	num -= (num/1000) * 1000;

	numStr[2] = num/100 + 0x30;
	num -= (num/100) * 100;

	numStr[3] = num/10 + 0x30;
	num -= (num/10) * 10;

	numStr[4] = num + 0x30;
}

void esp_init() {
	esp_indication();
	esp_timerIndication();
	USART2_Init();
}

void esp_indication() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
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

uint8_t esp_connect(const char* ssid_name, const char* ssid_pass) {
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

uint8_t esp_SetWiFiMode(const char* mode) {
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
	USART_PutString(ESP_CMD_SETMULTCONN, sizeStr(ESP_CMD_SETMULTCONN));
	USART_SendDataFromBuffer();

	return checkStatusCmd(curCMD);
}

uint8_t esp_createServer(uint16_t port) {
	esp_setMultConn();

	char strPort[5] = {0};
	intToStr(port, &strPort[0]);	//very specific function!!!

	curCMD = cmd_createServer;
	esp_status &= ~ESP_STATUS_BUSY;
	USART_PutString(ESP_CMD_CONF_TCP_SERV, sizeStr(ESP_CMD_CONF_TCP_SERV));
	USART_PutString(strPort, 5);
	USART_PutString("\r\n",2);

	USART_SendDataFromBuffer();
	return checkStatusCmd(curCMD);
}

uint8_t esp_processData(uint8_t doit) {
	uint8_t espByte;

	if (ProcessBuffer(&espByte) == 0) {
		return 0;
	}

	static uint8_t ptrCmd = 0;
	static uint8_t cmd[64] = {0};
	static uint8_t countCR = 0;	//var for count \r, when answer on cmd it's must be 2

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

	cmd[ptrCmd++] = espByte;
	cmd[ptrCmd] = 0x00;

	if (espByte == '\r') {
		++countCR;
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



		ptrCmd = 0;
		countCR = 0;
	}

	if (ptrCmd > 64) {
		ptrCmd = 0;
	}
//	cmd[ptrCmd++] = espByte;


	return 1;

}


