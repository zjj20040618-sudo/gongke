#ifndef HOST_BOOT_RNG_MAIN_H
#define HOST_BOOT_RNG_MAIN_H
#include <stdint.h>
typedef struct { uint32_t PLLCFGR, AHB2ENR, AHB2RSTR; } HostRcc;
typedef struct { uint32_t CR, SR, DR; } HostRng;
extern HostRcc host_rcc;
extern uint32_t SystemCoreClock;
HostRng *host_rng_regs(void);
uint32_t HAL_GetTick(void);
#define RCC (&host_rcc)
#define RNG host_rng_regs()
#define HSE_VALUE 8000000u
#define RCC_PLLCFGR_PLLSRC (1u << 22)
#define RCC_PLLCFGR_PLLSRC_HSE RCC_PLLCFGR_PLLSRC
#define RCC_PLLCFGR_PLLM 0x3fu
#define RCC_PLLCFGR_PLLN (0x1ffu << 6)
#define RCC_PLLCFGR_PLLQ (0xfu << 24)
#define RCC_AHB2ENR_RNGEN (1u << 6)
#define RCC_AHB2RSTR_RNGRST (1u << 6)
#define RNG_CR_RNGEN (1u << 2)
#define RNG_SR_DRDY 1u
#define RNG_SR_CECS (1u << 1)
#define RNG_SR_SECS (1u << 2)
#define RNG_SR_CEIS (1u << 5)
#define RNG_SR_SEIS (1u << 6)
#endif
