/*
 * hal_timebase_override.c - Provide HAL tick functions without HAL_Init()
 *
 * The STM32 HAL Ethernet driver uses HAL_GetTick() for timeouts. Since the
 * STEVE firmware does not call HAL_Init() and manages SysTick manually in
 * board.c, the HAL default weak implementations would never advance (uwTick
 * stays at zero).  We override the weak HAL functions and route them to the
 * board-level timebase so that HAL code that uses HAL_GetTick()/HAL_Delay()
 * works correctly without touching the HAL init sequence.
 */

#include "stm32h7xx_hal.h"
#include "board.h"

uint32_t HAL_GetTick(void)
{
    return board_get_systick_ms();
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    (void)TickPriority;
    return HAL_OK;
}

void HAL_Delay(uint32_t Delay)
{
    board_delay_ms(Delay);
}
