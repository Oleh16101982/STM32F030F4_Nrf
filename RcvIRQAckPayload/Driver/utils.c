#include "utils.h"
#include "main.h"
#include <string.h>

NRF24_TXResult tx_res;
uint8_t payload_length = NRF24_TX_PAYLOAD_WIDTH;
extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
extern unsigned char TxBuffer[20];
/*
void Timer4_Start(uint16_t tim_period)
{
	TIM_Cmd(TIM4, DISABLE);
	TIM_TimeBaseStructure.TIM_Period = tim_period;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);			
}
*/

void USART_SendStr(char *str, uint8_t flag_lrcr)
{
		uint8_t i;
	for(i = 0; i < 20; i++) {TxBuffer[i] = 0x00;}

	for (i = 0; i < strlen(str); i++) 
	{
		TxBuffer[i] = str[i];
	}
	if (flag_lrcr)
	{
		TxBuffer[strlen(str)] = '\r';
		TxBuffer[strlen(str) + 1] = '\n';
	}
	USART_ITConfig(USART, USART_IT_TXE, ENABLE);	
}

void NRF24_Config_transmit(void)
{
}

NRF24_TXResult NRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = NRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	NRF24_CE_LOW();

	// Transfer a data from the specified buffer to the TX FIFO
	NRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	NRF24_CE_HIGH();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = NRF24_GetStatus();
		if (status & (NRF24_FLAG_TX_DS | NRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	NRF24_CE_LOW();

	if (!wait) {
		// Timeout
		return NRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
////	UART_SendStr("[");
////	UART_SendHex8(status);
////	UART_SendStr("] ");

	// Clear pending IRQ flags
  NRF24_ClearIRQFlags(NRF24_MASK_STATUS_IRQ_RX | NRF24_MASK_STATUS_IRQ_TX | NRF24_MASK_STATUS_IRQ_RT);

	if (status & NRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return NRF24_TX_MAXRT;
	}

	if (status & NRF24_FLAG_TX_DS) {
		// Successful transmission
		return NRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	NRF24_FlushTX();

	return NRF24_TX_ERROR;
}

