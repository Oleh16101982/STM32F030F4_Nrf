/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx.h"
#include "utils.h"
// #include "main.h"

#define TXBUFFERSIZE   (countof(TxBuffer) - 1)
#define RXBUFFERSIZE   0x20

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
volatile uint8_t TxBuffer[] = "Hello. Stm32F030F4\n\r";
uint8_t RxBuffer[RXBUFFERSIZE];
uint8_t NbrOfDataToTransfer = TXBUFFERSIZE;
uint8_t NbrOfDataToRead = RXBUFFERSIZE;
__IO uint8_t TxCount = 0; 
__IO uint16_t RxCount = 0; 

extern uint16_t cnt_systick;
extern FlagStatus systick_delay_flag;
extern NRF24_RXResult pipe;
extern uint8_t NRF24_payload[NRF24_TX_PAYLOAD_WIDTH];
// extern uint8_t NRF24_ack_payload[NRF24_TX_PAYLOAD_WIDTH];
extern uint8_t payload_length;

extern uint8_t flag_tim17_update;
extern uint8_t flag_irq_rx;
extern uint8_t flag_irq_tx;
extern uint8_t flag_irq_rt;

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
	++cnt_systick;
	if (cnt_systick == SYSTICK_COUNT_MAX)
	{
		GPIO_WriteBit(MY_LED_RED_PORT, MY_LED_RED_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_RED_PORT, MY_LED_RED_PIN)));
		cnt_systick = 0;
		systick_delay_flag = SET;
	}		
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

void EXTI0_1_IRQHandler(void)
{
	uint8_t status;
	
	
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
//		USART_SendStr("IRQ - ",0);
  
    status = NRF24_GetStatus();
    if (status & NRF24_FLAG_RX_DR)
    {
GPIO_WriteBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN)));				
			flag_irq_rx = 1;
			NRF24_ClearIRQFlags(NRF24_MASK_STATUS_IRQ_RX);			
    }
		if (status & NRF24_FLAG_TX_DS)
      {
// GPIO_WriteBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN)));				
				USART_SendStr("TX.",1);
				flag_irq_tx = 1;
			NRF24_ClearIRQFlags(NRF24_MASK_STATUS_IRQ_TX);							
      }
      if (status & NRF24_FLAG_MAX_RT)
      {
				USART_SendStr("MAX RT.",1);
				flag_irq_rt = 1;
				NRF24_ClearIRQFlags(NRF24_MASK_STATUS_IRQ_RT);							
      }		
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

void EXTI4_15_IRQHandler(void)
{
	uint8_t status;
	
	
  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
		USART_SendStr("IRQ - ",0);
//    GPIO_WriteBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN)));
    status = NRF24_GetStatus();
    if (status & NRF24_FLAG_RX_DR)
    {
			USART_SendStr("RX.",1);
    }
		if (status & NRF24_FLAG_TX_DS)
      {
				USART_SendStr("TX.",1);
      }
      if (status & NRF24_FLAG_MAX_RT)
      {
				USART_SendStr("MAX RT.",1);
      }		
		
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
}

void TIM16_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET)
  {
//    GPIO_WriteBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_GREEN_PORT, MY_LED_GREEN_PIN)));
    TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
  }
}

void TIM17_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET)
  {
//    GPIO_WriteBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN, (BitAction)( 1 - GPIO_ReadOutputDataBit(MY_LED_YELLOW_PORT, MY_LED_YELLOW_PIN)));
		flag_tim17_update = 1;
    TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
		TIM_Cmd(TIM17, DISABLE);
  }
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer[RxCount++] = (USART_ReceiveData(USART) & 0x7F);

    if(RxCount == NbrOfDataToRead)
    {
      /* Disable the EVAL_COM1 Receive interrupt */
      USART_ITConfig(USART, USART_IT_RXNE, DISABLE);
    }
  }

  if(USART_GetITStatus(USART, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART, TxBuffer[TxCount++]);
while (!USART_GetFlagStatus(USART,USART_FLAG_TC)) {}; // wait for "Transmission Complete" flag cleared
    if(TxCount == NbrOfDataToTransfer)
    {
			TxCount = 0;
      /* Disable the EVAL_COM1 Transmit interrupt */
      USART_ITConfig(USART, USART_IT_TXE, DISABLE);
    }
		USART_ClearITPendingBit(USART, USART_IT_TXE);
  }
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
