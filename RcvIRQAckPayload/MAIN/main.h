
#define NRF24_PORT 	GPIOA
#define USART_PORT	GPIOA
#define	USART				USART1
#define NRF24_SPI		SPI1
// #define	TIM1_PORT		GPIOA
#define	MY_LED_RED_PORT	GPIOB
#define	MY_LED_YELLOW_PORT	GPIOA
#define	MY_LED_GREEN_PORT	GPIOA

#define NRF24_PIN_MOSI	GPIO_Pin_7
#define NRF24_PIN_MISO	GPIO_Pin_6
#define NRF24_PIN_SCK		GPIO_Pin_5
#define NRF24_PIN_CSN		GPIO_Pin_4
#define NRF24_PIN_CE		GPIO_Pin_1
#define NRF24_PIN_IRQ		GPIO_Pin_0

#define USART_PIN_TX	GPIO_Pin_9
#define USART_PIN_RX	GPIO_Pin_10

// #define TIM1_PIN_CH2	GPIO_Pin_9
// #define TIM1_PIN_CH3	GPIO_Pin_10

#define MY_LED_RED_PIN	GPIO_Pin_1
#define MY_LED_YELLOW_PIN	GPIO_Pin_2
#define MY_LED_GREEN_PIN	GPIO_Pin_3

#define SYSTICK_1ms	1000
#define SYSTICK_COUNT_MAX 1000

#define NRF24_WAIT_TIMEOUT (uint32_t)0x000FFFFF

#define TIM17_PERIOD_16us (uint16_t)1
#define TIM17_PERIOD_160us (uint16_t)19
#define TIM17_PERIOD_150ms (uint16_t)18749
#define TIM17_PERIOD_2ms  (uint16_t)249         //250 - 1

typedef enum {PWM_DOWN = 0, PWM_UP = !PWM_DOWN} PWM_Direction;
