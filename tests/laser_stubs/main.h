#ifndef LASER_HOST_MAIN_H
#define LASER_HOST_MAIN_H
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR;
} GPIO_TypeDef;
typedef struct {
    volatile uint32_t CR1, CCER, EGR, CNT, ARR, CCR1, CCR2, CCR3, CCR4;
} TIM_TypeDef;
typedef struct { uint32_t Period; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { unsigned id; } UART_HandleTypeDef;
extern GPIO_TypeDef host_gpio_a, host_gpio_b, host_gpio_c, host_gpio_e;
extern TIM_TypeDef host_tim11;
#define GPIOA (&host_gpio_a)
#define GPIOB (&host_gpio_b)
#define GPIOC (&host_gpio_c)
#define GPIOE (&host_gpio_e)
#define TIM11 (&host_tim11)
#define GPIO_PIN_0  (1u << 0u)
#define GPIO_PIN_1  (1u << 1u)
#define GPIO_PIN_8  (1u << 8u)
#define GPIO_PIN_9  (1u << 9u)
#define GPIO_PIN_10 (1u << 10u)
#define GPIO_PIN_11 (1u << 11u)
#define GPIO_PIN_12 (1u << 12u)
#define GPIO_PIN_13 (1u << 13u)
#define GPIO_PIN_14 (1u << 14u)
#define GPIO_PIN_15 (1u << 15u)
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define TIM_CHANNEL_3 8u
#define TIM_CHANNEL_4 12u
#define TIM_CHANNEL_ALL 0x3cu
#define TIM_EGR_UG 1u
#define TIM_CCER_CC1E 1u
#define TIM_CR1_CEN 1u

#define LASER_PWMA_Pin GPIO_PIN_9
#define LASER_PWMA_GPIO_Port GPIOB
#define LASER_AIN1_Pin GPIO_PIN_0
#define LASER_AIN1_GPIO_Port GPIOB
#define LASER_AIN2_Pin GPIO_PIN_1
#define LASER_AIN2_GPIO_Port GPIOB
#define LASER_STBY_Pin GPIO_PIN_12
#define LASER_STBY_GPIO_Port GPIOC

extern unsigned host_clock_enable_mask;
#define __HAL_RCC_GPIOB_CLK_ENABLE() (host_clock_enable_mask |= 1u)
#define __HAL_RCC_GPIOC_CLK_ENABLE() (host_clock_enable_mask |= 2u)
#define __HAL_RCC_TIM11_CLK_ENABLE() (host_clock_enable_mask |= 4u)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __set_PRIMASK(uint32_t mask);
void Error_Handler(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, GPIO_PinState);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void host_compare(TIM_HandleTypeDef *, uint32_t, uint32_t);
#define __HAL_TIM_SET_COMPARE(h, c, v) host_compare((h), (c), (v))
#define __HAL_TIM_SET_COUNTER(h, v) ((h)->Instance->CNT = (v))
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#endif
