/* Synthetic CMSIS-register adapter checks, not hardware entropy evidence. */
#include <stdio.h>
#include <string.h>
#include "boot_session_rng.h"
HostRcc host_rcc;
uint32_t SystemCoreClock;
static HostRng regs;
static uint32_t tick, tick_step, words[3];
static unsigned calls, no_ready, fail_call, fail_bits;
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"boot RNG line%d: %s\n",__LINE__,#c); return 1; } } while (0)
uint32_t HAL_GetTick(void) { uint32_t out=tick; tick+=tick_step; return out; }
HostRng *host_rng_regs(void)
{
    /* CR enable is call0. Each successful word: SR, DR, SR. Call10 is
     * shutdown CR. Model DRDY consumption with that explicit access trace. */
    unsigned n=calls++;
    if (n>=1u && n<=9u) regs.DR=words[(n-1u)/3u];
    regs.SR=no_ready ? 0u : RNG_SR_DRDY;
    if (fail_call && n==fail_call) regs.SR|=fail_bits;
    return &regs;
}
static void reset_fixture(void)
{
    memset(&host_rcc,0,sizeof host_rcc); memset(&regs,0,sizeof regs);
    host_rcc.PLLCFGR=RCC_PLLCFGR_PLLSRC_HSE|8u|(336u<<6)|(7u<<24);
    SystemCoreClock=168000000u; tick=0u; tick_step=1u;
    words[0]=0xaabbccddu; words[1]=0x12345678u; words[2]=0x90abcdefu;
    calls=no_ready=fail_call=fail_bits=0u;
}
static int all_zero(const uint8_t out[8])
{ for (unsigned i=0u;i<8u;++i) if(out[i]) return 0; return 1; }
int main(void)
{
    uint8_t out[8], expected[8]={0x78,0x56,0x34,0x12,0xef,0xcd,0xab,0x90};
    reset_fixture(); CHECK(boot_session_nonce(out)); CHECK(!memcmp(out,expected,8u));
    CHECK(calls==11u && !regs.CR && !host_rcc.AHB2ENR && !host_rcc.AHB2RSTR);
    reset_fixture(); host_rcc.PLLCFGR=(host_rcc.PLLCFGR&~RCC_PLLCFGR_PLLQ)|(4u<<24);
    CHECK(!boot_session_nonce(out) && all_zero(out) && !calls);
    reset_fixture(); SystemCoreClock=84000000u;
    CHECK(!boot_session_nonce(out) && all_zero(out) && !calls);
    reset_fixture(); host_rcc.AHB2ENR=RCC_AHB2ENR_RNGEN|1u; regs.CR=99u;
    CHECK(!boot_session_nonce(out) && all_zero(out) && !calls);
    CHECK(host_rcc.AHB2ENR==(RCC_AHB2ENR_RNGEN|1u) && regs.CR==99u);
    const unsigned errors[]={RNG_SR_CECS,RNG_SR_SECS,RNG_SR_CEIS,RNG_SR_SEIS};
    for (unsigned i=0u;i<4u;++i) for(unsigned after=0u;after<2u;++after) {
        reset_fixture(); fail_call=after ? 3u : 1u; fail_bits=errors[i];
        CHECK(!boot_session_nonce(out) && all_zero(out) && !regs.CR && !host_rcc.AHB2ENR);
    }
    reset_fixture(); words[1]=words[0]; CHECK(!boot_session_nonce(out) && all_zero(out));
    reset_fixture(); words[2]=words[1]; CHECK(!boot_session_nonce(out) && all_zero(out));
    reset_fixture(); words[1]=words[2]=0u; CHECK(!boot_session_nonce(out) && all_zero(out));
    reset_fixture(); no_ready=1u; tick=0xfffffff0u;
    CHECK(!boot_session_nonce(out) && all_zero(out) && calls<30u && !regs.CR && !host_rcc.AHB2ENR);
    reset_fixture(); no_ready=1u; tick_step=0u;
    CHECK(!boot_session_nonce(out) && all_zero(out) && calls==2000002u && !regs.CR && !host_rcc.AHB2ENR);
    puts("boot RNG synthetic adapter: byte order/clock guard/owner protection/error bits/read errors/duplicates/zero/bounded stalled-tick/wrap/shutdown PASS (not physical RNG validation)");
    return 0;
}
