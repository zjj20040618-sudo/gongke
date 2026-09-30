#ifndef HOST_TEST_MAIN_H
#define HOST_TEST_MAIN_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
#endif
