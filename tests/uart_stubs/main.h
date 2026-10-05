#ifndef UART_CALLBACK_HOST_MAIN_H
#define UART_CALLBACK_HOST_MAIN_H
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2 } HAL_StatusTypeDef;
typedef struct {
    uint32_t RxState, ErrorCode;
    uint8_t *rx;
    unsigned arm_calls, fail_next_arm, rx_enabled;
    unsigned ore_flag, rxne_flag, dr_reads, clear_calls;
    uint8_t dr;
} UART_HandleTypeDef;

#define HAL_UART_STATE_READY   0x20u
#define HAL_UART_STATE_BUSY_RX 0x22u
#define HAL_UART_ERROR_NONE    0x00u
#define HAL_UART_ERROR_PE      0x01u
#define HAL_UART_ERROR_NE      0x02u
#define HAL_UART_ERROR_FE      0x04u
#define HAL_UART_ERROR_ORE     0x08u
#define UART_FLAG_ORE          0x08u

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);
void host_uart_clear_ore(UART_HandleTypeDef *);
#define __HAL_UART_GET_FLAG(h, flag) ((void)(flag), (h)->ore_flag != 0u)
#define __HAL_UART_CLEAR_OREFLAG(h) host_uart_clear_ore(h)
#endif
