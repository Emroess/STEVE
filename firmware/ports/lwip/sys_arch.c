/**
  * @file    sys_arch.c
  * @author  STEVE firmware team
  * @brief   lwIP system abstraction layer for bare-metal STM32H7
  */

/* Includes ------------------------------------------------------------------*/
#include "lwipopts.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/stats.h"
#include "lwip/debug.h"
#include "lwip/sys.h"
#include "lwip/err.h"
#include "lwip/def.h"
#include "arch/cc.h"
#include "board.h"
#include "stm32h7xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYS_ARCH_TIMEOUT 0xFFFFFFFFUL
#define ERR_MEM -1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Returns the current time in milliseconds
  * @param  None
  * @retval Current time in milliseconds
  */
u32_t sys_now(void)
{
  return board_get_systick_ms();
}

/**
  * @brief  Initializes the sys_arch layer
  * @param  None
  * @retval None
  */
void sys_init(void)
{
  /* Nothing to do for bare-metal */
}

/**
  * @brief  Protects the critical section
  * @param  None
  * @retval None
  */
sys_prot_t sys_arch_protect(void)
{
  /* Disable interrupts */
  __disable_irq();
  return 0;
}

/**
  * @brief  Unprotects the critical section
  * @param  pval: protection value
  * @retval None
  */
void sys_arch_unprotect(sys_prot_t pval)
{
  /* Enable interrupts */
  (void)pval;
  __enable_irq();
}