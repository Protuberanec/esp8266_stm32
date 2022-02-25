#ifndef	_usart_h_
#define	_usart_h_

#include	<stm32f0xx.h>
#include	"buffer.h"

#define	SIZE_BUF_TX	256
#define	SIZE_BUF_RX	1024


void USART1_init();
void USART1_sendData(uint8_t data);

void USART2_Init();
uint8_t USART_GetData(void);

void USART_PutCharToBuff(uint8_t data);	//put buffer
void USART_PutString(const uint8_t* data, uint16_t size);
void USART_SendDataFromBuffer(void);	//send to pc

uint8_t clearRxBuffer();

void USART2_SendDataViaDMA(const uint8_t* data, uint16_t size);






uint8_t ProcessBuffer(uint8_t *data);

#endif
