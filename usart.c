#include	"usart.h"

static cBuffer buffer_tx;
static cBuffer buffer_rx;
static uint8_t data_tx[SIZE_BUF_TX];
static uint8_t data_rx[SIZE_BUF_RX];


void USART2_IRQHandler() {
	if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		uint8_t temp_data = (uint16_t)(USART2->RDR & 0x1FF);
		USART1_sendData(temp_data);
		bufferAddToEnd(&buffer_rx, temp_data);
	}

	if ((USART2->ISR & USART_ISR_TXE) == USART_ISR_TXE) {
		if (!buffer_tx.datalength) {
			USART2->CR1 &= ~USART_CR1_TXEIE;
			return;
		}
		USART2->TDR = bufferGetFromFront(&buffer_tx);
	}
}

void USART1_init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER9_1;
	GPIOA->AFR[1] |= 1 << 4;

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 |= USART_CR1_TE;	//transmit and received enabled
	USART1->BRR = SystemCoreClock / 256000;
	USART1->CR3 |= USART_CR3_OVRDIS;
	USART1->CR1 |= USART_CR1_UE;	//enable usart
}

void USART1_sendData(uint8_t data) {
	USART1->TDR = data;
}

void USART2_Init() {
	bufferInit(&buffer_tx, &data_tx[0], SIZE_BUF_TX);
	bufferInit(&buffer_rx, &data_rx[0], SIZE_BUF_RX);

	//setup pins
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1;
	GPIOA->AFR[0] |= 0x01 << 8 | 0x01 << 12;	//AF pins for usart...
	//setup usart
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;	//transmit and received enabled
	USART2->BRR = SystemCoreClock / 115200;
	USART2->CR1 |= 	/*USART_CR1_TCIE | */ /*USART_CR1_TXEIE |*/ USART_CR1_RXNEIE;//set interrupts
	USART2->CR3 |= USART_CR3_OVRDIS;

	NVIC_SetPriority(USART2_IRQn, 2);
	NVIC_EnableIRQ(USART2_IRQn);

	USART2->CR1 |= USART_CR1_UE;	//enable usart
	__enable_irq();

	USART1_init();
}

/*function to get data from the usart and put data to buffer*/
uint8_t USART_GetData(void) {
	if ( (USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE )
	{

		return 1;
	}
	return 0;
}

/*function to put data to the usart*/
void USART_PutCharToBuff(uint8_t data) {
	bufferAddToEnd(&buffer_tx, data);
}

void USART2_SendDataViaDMA(const uint8_t* data, uint16_t size) {

}

void USART_SendDataFromBuffer(void) {
	if (!buffer_tx.datalength) {
		USART2->CR1 &= ~USART_CR1_TXEIE;
		return;
	}

	if ( (USART2->ISR & USART_ISR_TXE) == USART_ISR_TXE)
	{
		if ((USART2->ISR & USART_ISR_TC) == USART_ISR_TC)
		{
			USART2->TDR = bufferGetFromFront(&buffer_tx);
			USART2->CR1 |= USART_CR1_TXEIE;
		}
	}
}

void USART_PutString(const uint8_t* data, uint16_t size) {

	for (int i = 0; i < size; i++) {
		USART_PutCharToBuff(data[i]);
	}
}

uint8_t clearRxBuffer() {
	bufferFlush(&buffer_rx);
}

uint8_t ProcessBuffer(uint8_t* data) {
	if (!buffer_rx.datalength) {
		return 0;
	}

	*data = bufferGetFromFront(&buffer_rx);

	return 1;
}
