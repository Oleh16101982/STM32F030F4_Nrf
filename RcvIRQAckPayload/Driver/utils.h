#include "stm32f0xx.h"
#include "stm32f0xx_misc.h"
#include "nrf24.h"

void Timer4_Start(uint16_t tim_period);
void USART_SendStr(char *str, uint8_t flag_lrcr);
void NRF24_Config_transmit(void);
NRF24_TXResult NRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);

