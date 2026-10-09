#ifndef BOOT_SESSION_RNG_H
#define BOOT_SESSION_RNG_H

/* Only included by robot.c in the opt-in session build. No Flash writes and
 * no time/UID fallback: a repeated boot nonce would defeat old-packet isolation.
 * STM32F407 RM0090: PLL48CLK for RNG must be <=48 MHz and >HCLK/16.
 * main.c selects PLLQ=7 ONLY for this build (8 MHz HSE, M8/N336 ->48 MHz).
 * The CPU/APB/timer clocks are unchanged. Cube regeneration must retain it. */
#include "main.h"
#include <string.h>

static int boot_rng_word(uint32_t *out)
{
    const uint32_t start = HAL_GetTick();
    for (uint32_t tries = 0u; tries < 2000000u; ++tries) {
        uint32_t status = RNG->SR;
        if (status & (RNG_SR_CECS | RNG_SR_SECS | RNG_SR_CEIS | RNG_SR_SEIS)) return 0;
        if (status & RNG_SR_DRDY) {
            *out = RNG->DR; /* DR read clears DRDY on hardware. */
            return !(RNG->SR & (RNG_SR_CECS | RNG_SR_SECS | RNG_SR_CEIS | RNG_SR_SEIS));
        }
        if ((uint32_t)(HAL_GetTick() - start) >= 25u) return 0;
    }
    return 0; /* Also bounded when the early HAL tick is not advancing. */
}

static int boot_session_nonce(uint8_t out[8])
{
    uint32_t discard, low, high;
    int ok;
    memset(out, 0, 8u);
    /* Reject an altered/regenerated clock tree rather than use out-of-spec RNG.
     * These are the project's verified 8MHz HSE /168MHz core parameters. */
    if (HSE_VALUE != 8000000u || SystemCoreClock != 168000000u ||
        (RCC->PLLCFGR & (RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM |
                        RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLQ)) !=
        (RCC_PLLCFGR_PLLSRC_HSE | 8u | (336u << 6) | (7u << 24))) return 0;
    if (RCC->AHB2ENR & RCC_AHB2ENR_RNGEN) return 0; /* Do not reset someone else's RNG. */
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB2ENR;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_RNGRST;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
    RNG->CR = RNG_CR_RNGEN; /* No RNG interrupt/TX in ISR. */
    ok = boot_rng_word(&discard) && boot_rng_word(&low) && boot_rng_word(&high);
    RNG->CR = 0u;
    RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
    if (!ok || discard == low || low == high || (low == 0u && high == 0u)) return 0;
    for (unsigned i = 0u; i < 4u; ++i) {
        out[i] = (uint8_t)(low >> (8u * i));
        out[4u + i] = (uint8_t)(high >> (8u * i));
    }
    return 1;
}
#endif
