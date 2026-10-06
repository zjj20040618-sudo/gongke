/* Real arm.c with a deterministic virtual DWT, not a physical waveform test. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "tim.h"

typedef struct { uint32_t DEMCR; } HostCoreDebug;
typedef struct { uint32_t CTRL, CYCCNT; } HostDwt;
static HostCoreDebug host_debug;
static HostDwt host_dwt;
static unsigned dwt_reads;
uint32_t SystemCoreClock = 168000000u;

/* Each access advances one simulated microsecond, so busy_us cannot hang.
 * One start read plus 20 elapsed reads checks the existing busy_us(20) call. */
static HostDwt *host_dwt_access(void)
{
    assert(++dwt_reads < 1000u);
    host_dwt.CYCCNT += SystemCoreClock / 1000000u;
    return &host_dwt;
}
#define CoreDebug (&host_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24u)
#define DWT host_dwt_access()
#define DWT_CTRL_CYCCNTENA_Msk 1u

GPIO_TypeDef host_gpio_a;
static TIM_TypeDef host_tim12;
TIM_HandleTypeDef htim12 = { &host_tim12, { 9999u } };
typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin, cycles;
    GPIO_PinState state;
} PinWrite;
static PinWrite writes[8];
static unsigned write_count;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state)
{
    assert(write_count < sizeof(writes) / sizeof(writes[0]));
    writes[write_count++] = (PinWrite){ port, pin, host_dwt.CYCCNT, state };
}
void host_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
    assert(timer == &htim12 && channel == TIM_CHANNEL_2);
    timer->Instance->CCR2 = value;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(timer == &htim12 && channel == TIM_CHANNEL_2);
    return HAL_OK;
}
void Error_Handler(void) { assert(!"unexpected Error_Handler"); }

/* Match the real main.h/Cube labels; the runner checks that source contract. */
#define ARM_AXIS0_DIR_Pin GPIO_PIN_9
#define ARM_AXIS0_DIR_GPIO_Port GPIOA
#define ARM_AXIS0_STEP_Pin GPIO_PIN_10
#define ARM_AXIS0_STEP_GPIO_Port GPIOA
#define ARM_AXIS1_DIR_Pin GPIO_PIN_11
#define ARM_AXIS1_DIR_GPIO_Port GPIOA
#define ARM_AXIS1_STEP_Pin GPIO_PIN_12
#define ARM_AXIS1_STEP_GPIO_Port GPIOA
#include "../App/arm.c"

static void check_write(unsigned index, uint32_t pin, GPIO_PinState state)
{
    assert(writes[index].port == GPIOA);
    assert(writes[index].pin == pin && writes[index].state == state);
}

static void check_axis(int axis, uint32_t dir_pin, uint32_t step_pin)
{
    write_count = 0u;
    arm_stepper_dir(axis, 0);
    arm_stepper_dir(axis, 1);
    assert(write_count == 2u);
    check_write(0u, dir_pin, GPIO_PIN_RESET);
    check_write(1u, dir_pin, GPIO_PIN_SET);

    /* Also exercise the unsigned cycle-counter wrap used by production. */
    for (unsigned wrap = 0u; wrap < 2u; ++wrap) {
        write_count = dwt_reads = 0u;
        host_dwt.CYCCNT = wrap ? UINT32_MAX - 100u : 0u;
        arm_stepper_step(axis);
        assert(write_count == 2u && dwt_reads == 21u);
        check_write(0u, step_pin, GPIO_PIN_RESET);
        check_write(1u, step_pin, GPIO_PIN_SET);
        assert((uint32_t)(writes[1].cycles - writes[0].cycles)
               == 21u * (SystemCoreClock / 1000000u));
    }
}

int main(void)
{
    arm_init();
    assert((host_debug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u);
    assert((host_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
    assert(write_count == 2u);
    check_write(0u, GPIO_PIN_9, GPIO_PIN_RESET);
    check_write(1u, GPIO_PIN_11, GPIO_PIN_RESET);
    check_axis(0, GPIO_PIN_9, GPIO_PIN_10);
    check_axis(1, GPIO_PIN_11, GPIO_PIN_12);
    write_count = dwt_reads = 0u;
    arm_stepper_dir(-1, 0);
    arm_stepper_dir(ARM_STEPPER_NUM, 1);
    arm_stepper_step(-1);
    arm_stepper_step(ARM_STEPPER_NUM);
    assert(write_count == 0u && dwt_reads == 0u);
    puts("arm stepper: axis0 STEP PA10/DIR PA9; axis1 STEP PA12/DIR PA11; low/high pulse and virtual 20us passed");
    return 0;
}
