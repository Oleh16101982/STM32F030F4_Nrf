#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_misc.h"
#include "main.h"
#include "utils.h"
// #include "nrf24.h"


// #include "main.h"
static const uint8_t NRF24_ADDR[] = { 'O', 'B', 'A' , 'R', 'A'};
uint8_t NRF24_payload[NRF24_TX_PAYLOAD_WIDTH];
uint8_t NRF24_ack_payload[NRF24_TX_PAYLOAD_WIDTH] = {0};
extern uint8_t payload_length;
NRF24_RXResult pipe;
uint16_t timer_period = 0;
volatile uint32_t tim17_period = TIM17_PERIOD_16us;
volatile uint8_t flag_tim17_update = 0;
volatile uint16_t channel1_pulse = 0x0F;
volatile uint16_t channel2_pulse = 0xF0;
volatile PWM_Direction channel1_direction = PWM_UP;
volatile PWM_Direction channel2_direction = PWM_DOWN;
volatile FlagStatus systick_delay_flag = RESET;

volatile uint8_t flag_irq_rx = 0;
volatile uint8_t flag_irq_tx = 0;
volatile uint8_t flag_irq_rt = 0;

uint16_t cnt_systick = 0;
GPIO_InitTypeDef GPIO_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
// TIM_OCInitTypeDef  TIM_OCInitStructure;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
SPI_InitTypeDef   SPI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;


ErrorStatus HSEStartUpStatus;
extern uint8_t TxBuffer[];
void RCC_Config(void)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if (HSEStartUpStatus == SUCCESS)
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLKConfig(RCC_HCLK_Div1);
        //ADC CLK
        RCC_ADCCLKConfig(RCC_ADCCLK_HSI14);
        /* PLLCLK = 8MHz * 3 = 24 MHz */
        RCC_PLLConfig(RCC_PLLSource_PREDIV1 , RCC_PLLMul_4); //RCC_PLLSource_HSE_Div1, RCC_PLLMul_2); 4 * 8MHz = 32MHz
        /* Enable PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//////			RCC_RTCCLKCmd(ENABLE);
        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08) {}
				RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB , ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_SYSCFG | RCC_APB2Periph_TIM16, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);					
	}
}



void NVIC_Config(void)
{
  /* Enable the USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
	
		
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);		

  NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
	
}

void GPIO_Config(void)
{
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	
	GPIO_InitStructure.GPIO_Pin = MY_LED_RED_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(MY_LED_RED_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = MY_LED_YELLOW_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(MY_LED_YELLOW_PORT, &GPIO_InitStructure);		
	
	GPIO_InitStructure.GPIO_Pin = MY_LED_GREEN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(MY_LED_GREEN_PORT, &GPIO_InitStructure);		
	
/*	
	GPIO_InitStructure.GPIO_Pin = TIM1_PIN_CH2 | TIM1_PIN_CH3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(TIM1_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(TIM1_PORT, GPIO_PinSource9, GPIO_AF_2);	
	GPIO_PinAFConfig(TIM1_PORT, GPIO_PinSource10, GPIO_AF_2);
*/	
	GPIO_InitStructure.GPIO_Pin = USART_PIN_TX | USART_PIN_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(USART_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(USART_PORT, GPIO_PinSource9, GPIO_AF_1);	
	GPIO_PinAFConfig(USART_PORT, GPIO_PinSource10, GPIO_AF_1);	
	
// GPIO SPI Config
	GPIO_InitStructure.GPIO_Pin = NRF24_PIN_MISO | NRF24_PIN_MOSI | NRF24_PIN_SCK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(NRF24_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(NRF24_PORT, GPIO_PinSource7, GPIO_AF_0);	
	GPIO_PinAFConfig(NRF24_PORT, GPIO_PinSource6, GPIO_AF_0);	
	GPIO_PinAFConfig(NRF24_PORT, GPIO_PinSource5, GPIO_AF_0);

	GPIO_InitStructure.GPIO_Pin = NRF24_PIN_CE | NRF24_PIN_CSN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(NRF24_PORT, &GPIO_InitStructure);	

}
/*
void Tim1_Config(void)
{	
  timer_period = (SystemCoreClock / 17570 ) - 1;
  channel2_pulse = 0; // (uint16_t) (((uint32_t) 5 * (timer_period - 1)) / 10);
//  channel2_pulse = (uint16_t) (((uint32_t) 375 * (timer_period - 1)) / 1000);
  channel1_pulse = (uint16_t)0; // (((uint32_t) 375 * (timer_period - 1)) / 1000);
	
	
  TIM_TimeBaseStructure.TIM_Prescaler = 313;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 0xFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = channel1_pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = channel2_pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);	
}
*/
void USART_Config(void)
{
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART, &USART_InitStructure);
	
  USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART, USART_IT_TXE, DISABLE);	
  USART_Cmd(USART, ENABLE);	
}

void SPI_Config(void)
{
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0x07;
	SPI_Init(NRF24_SPI, &SPI_InitStructure);
	SPI_RxFIFOThresholdConfig(NRF24_SPI, SPI_RxFIFOThreshold_QF);
	
	
	SPI_Cmd(NRF24_SPI, ENABLE);	
}

void EXTI0_Config(void)
{
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //  GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 		//   GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource14);	
	
  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	
  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
	
  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	

  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);		
}

void Tim16_Config(void)
{
	TIM_TimeBaseStructure.TIM_Prescaler = 32000 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM16, ENABLE);
	
}

// For create delay for work NRF24l01

void Tim17_Config(void)
{
	TIM_TimeBaseStructure.TIM_Prescaler = 256;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = tim17_period;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
//	TIM_Cmd(TIM17, DISABLE);
	
}

int main(void)
{
	uint8_t status = 0;
	RCC_Config();
	
	SysTick_Config(SystemCoreClock / SYSTICK_1ms);
	GPIO_Config();
	GPIO_WriteBit(MY_LED_RED_PORT, MY_LED_RED_PIN, Bit_SET);
	GPIO_WriteBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN, Bit_RESET);
	GPIO_WriteBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN, Bit_RESET);
//	Tim1_Config();
	Tim16_Config();
	Tim17_Config();	
	USART_Config();
	SPI_Config();
	NVIC_Config();
	EXTI0_Config();
	


	
	NRF24_CE_LOW();
	NRF24_CSN_HIGH();
// delay power on reset for NRF24l01 need more than 100ms delay
	flag_tim17_update = 0;
	tim17_period = TIM17_PERIOD_150ms;
	TIM_TimeBaseStructure.TIM_Period = tim17_period;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM17, ENABLE);
	while(!flag_tim17_update) {}
	flag_tim17_update = 0;	

	USART_SendStr("Hello.Stm32", 1);
//	USART_ITConfig(USART, USART_IT_TXE, ENABLE);
    USART_SendStr("Check: ", 0);
/*		
	TxBuffer[0] = 0xFF;
	TxBuffer[1] = 0xFF;
	TxBuffer[2] = NRF24_readReg(0x07);
	TxBuffer[3] = 0xFE;
	TxBuffer[3] = 0xFE;
	USART_ITConfig(USART, USART_IT_TXE, ENABLE);
*/	
    if (!NRF24_Check()) 
		{
    	USART_SendStr("FAIL", 1);
    	while (1);
    }

	USART_SendStr("OK", 1);	
	NRF24_Init();
	NRF24_SetRFChannel(40);		
	NRF24_SetDataRate(NRF24_DR_2Mbps);
	NRF24_SetCRCScheme(NRF24_CRC_2byte);
	NRF24_SetAddrWidth(NRF24_TX_ADDRESS_WIDTH);	
	NRF24_SetAddr(NRF24_PIPE1, NRF24_ADDR);
	NRF24_SetRXPipe(NRF24_PIPE1, NRF24_AA_ON, NRF24_TX_PAYLOAD_WIDTH);
	NRF24_SetTXPower(NRF24_TXPWR_0dBm);
  NRF24_EnableAA(NRF24_PIPE0);		
  NRF24_SetOperationalMode(NRF24_MODE_RX);		

	NRF24_SetDPL();
	NRF24_SetAckPay();	
//	NRF24_SetDynAsk();
/*		
	status = NRF24_readReg(NRF24_FEATURE);		
	if (!status)
	{		
		NRF24_SetActivateDPL();
		NRF24_SetDPL();
		NRF24_SetAckPay();	
//		NRF24_SetDynAsk();		
	}	
*/	
	NRF24_SetDplPipe(0x00, 0x01); 
	NRF24_SetDplPipe(0x01, 0x01);		
  NRF24_ClearIRQFlags(NRF24_MASK_STATUS_IRQ_RX | NRF24_MASK_STATUS_IRQ_TX | NRF24_MASK_STATUS_IRQ_RT);
		
	
		
  NRF24_SetPowerMode(NRF24_PWR_UP);
// Put the transceiver to the RX mode
  	
	flag_tim17_update = 0;
	tim17_period = TIM17_PERIOD_2ms;
	TIM_TimeBaseStructure.TIM_Period = tim17_period;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM17, ENABLE);
	while(!flag_tim17_update) {}
	flag_tim17_update = 0;	
		
	status = NRF24_readReg(NRF24_FEATURE);
	USART_SendData(USART, '('); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, status); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, ')'); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};		
	
	status = NRF24_readReg(NRF24_EN_AA);
	USART_SendData(USART, '['); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, status); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, ']'); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
		
//	NRF24_WriteAckPayload(NRF24_ack_payload, &payload_length); 			
		
	NRF24_CE_HIGH();
	
	NRF24_ack_payload[0] = 'A';
	NRF24_ack_payload[1] = 'Z';	
	NRF24_WriteAckPayload(NRF24_ack_payload, payload_length); 		
	
	status = NRF24_readReg(NRF24_FIFO_STATUS);
	USART_SendData(USART, '['); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, status); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, ']'); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
//	NRF24_WriteAckPayload(NRF24_ack_payload, payload_length);		
	while(1)
	{
//		while(!systick_delay_flag) {}
		if (systick_delay_flag) {systick_delay_flag = RESET;}
//		GPIO_WriteBit(MY_LED_RED_PORT, MY_LED_RED_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_RED_PORT, MY_LED_RED_PIN)));		
		if (flag_irq_rx)
		{		
			USART_SendStr("Rx", 1);
			if (NRF24_GetStatus_RXFIFO() != NRF24_STATUS_RXFIFO_EMPTY) 
			{
NRF24_WriteAckPayload(NRF24_ack_payload, payload_length);				
	status = NRF24_readReg(NRF24_FIFO_STATUS);
	USART_SendData(USART, '['); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, status); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};
	USART_SendData(USART, ']'); 
	while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {};				
// Get a payload from the transceiver
				pipe = NRF24_ReadPayload(NRF24_payload, &payload_length);
//			GPIO_WriteBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN)));
// Clear all pending IRQ flags
//				NRF24_ClearIRQFlags();
// Print a payload contents to UART
				NRF24_ack_payload[0] = NRF24_payload[1];
				NRF24_ack_payload[1] = NRF24_payload[0];				
				if (NRF24_payload[0] == 0x37)
				{
					GPIO_WriteBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN)));
				}		
			}
			flag_irq_rx = 0;			
		}		
		if (flag_irq_tx)
		{		
			USART_SendStr("flag Tx", 1);
			flag_irq_tx = 0;			
		}					
	};
}

