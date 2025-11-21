/*
 * system_stm32h7xx.c - Minimal CMSIS system initialization
 *
 * Per IMPROVE_FIRMWARE_PLAN.md Option A (CLOCK_CONFIG_OWNER_BOARD):
 * - SystemInit() does ONLY FPU setup
 * - board_init() in board.c owns ALL clock and power configuration
 *
 * This keeps the CMSIS requirement that SystemInit() exists and is called
 * from startup code, but delegates all hardware configuration to board.c.
 *
 * Based on STMicroelectronics CMSIS implementation
 * Modified for bare metal firmware compliance
 */

#include <stdint.h>
#include "stm32h7xx.h"

/* Check that HSE value matches board configuration */
#if !defined(HSE_VALUE)
#define HSE_VALUE 25000000U  /* 25 MHz HSE on NUCLEO-H753ZI */
#endif

/*
 * SystemCoreClock - System clock frequency in Hz
 *
 * This variable is updated by board_init() after clock configuration.
 * Initial value is HSI (64 MHz) which is the default after reset.
 */
uint32_t SystemCoreClock = 64000000U;
uint32_t SystemD2Clock = 64000000U;
const uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

/*
 * ExitRun0Mode - Exit RUN0 mode and enable LDO regulator
 *
 * Called early from startup code before SystemInit().
 * Configures the power supply to use LDO (Low DropOut regulator).
 *
 * For NUCLEO-H753ZI board which uses LDO power supply.
 *
 * NOTE: This function must NOT call any functions or use any variables
 * because it runs before .data/.bss initialization!
 */
void
ExitRun0Mode(void)
{
	/* Do nothing - use default power configuration after reset */
	/* LDO should already be enabled by hardware after reset */
}

/*
 * SystemInit - Minimal system initialization
 *
 * Per IMPROVE_FIRMWARE_PLAN.md:
 * - FPU setup only (if FPU is present)
 * - NO clock configuration
 * - NO power configuration
 * - NO peripheral configuration
 *
 * All hardware configuration is done in board_init().
 */
void
SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	/* Enable FPU (Floating Point Unit) */
	/* CPACR: Coprocessor Access Control Register */
	/* CP10 and CP11 Full Access (bits 20-23) */
	SCB->CPACR |= ((3UL << 20U) | (3UL << 22U));
#endif

	/* Reset the RCC clock configuration to default reset state */
	/* Set HSION bit (should already be on after reset) */
	RCC->CR |= RCC_CR_HSION;

	/* Reset CFGR register to default */
	RCC->CFGR = 0x00000000U;

	/* Reset HSEON, CSSON, CSION, HSI48ON, CSIKERON, PLL1ON, PLL2ON, PLL3ON bits */
	RCC->CR &= 0xEAF6ED7FU;

	/* Reset D1CFGR register */
	RCC->D1CFGR = 0x00000000U;

	/* Reset D2CFGR register */
	RCC->D2CFGR = 0x00000000U;

	/* Reset D3CFGR register */
	RCC->D3CFGR = 0x00000000U;

	/* Reset PLLCKSELR register */
	RCC->PLLCKSELR = 0x00000000U;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x00000000U;

	/* Reset PLL1DIVR register */
	RCC->PLL1DIVR = 0x00000000U;

	/* Reset PLL1FRACR register */
	RCC->PLL1FRACR = 0x00000000U;

	/* Reset PLL2DIVR register */
	RCC->PLL2DIVR = 0x00000000U;

	/* Reset PLL2FRACR register */
	RCC->PLL2FRACR = 0x00000000U;

	/* Reset PLL3DIVR register */
	RCC->PLL3DIVR = 0x00000000U;

	/* Reset PLL3FRACR register */
	RCC->PLL3FRACR = 0x00000000U;

	/* Reset HSEBYP bit */
	RCC->CR &= 0xFFFBFFFFU;

	/* Disable all interrupts */
	RCC->CIER = 0x00000000U;

	/*
	 * Disable FMC bank1 (enabled after reset on STM32H7)
	 * This prevents CPU speculation issues that can block other masters
	 * Per ST reference implementation
	 */
	FMC_Bank1_R->BTCR[0] = 0x000030D2U;

	/*
	 * DO NOT configure clocks, PLLs, VOS, or MPU here
	 * All of that is in board_init()
	 */
}

/*
 * SystemCoreClockUpdate - Update SystemCoreClock variable
 *
 * This function is called by board_init() after clock configuration.
 * It can also be called by application code if clock is changed at runtime.
 *
 * Note: In this firmware, clock configuration is static and done once
 * in board_init(), so this function is primarily for CMSIS compliance.
 */
void
SystemCoreClockUpdate(void)
{
	uint32_t sysclk_source;
	uint32_t pllm;
	uint32_t plln;
	uint32_t pllp;
	uint32_t sysclk;

	/* Get system clock source (SW field in RCC_CFGR) */
	sysclk_source = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;

	switch (sysclk_source) {
	case 0x0:  /* HSI used as system clock */
		sysclk = 64000000U;
		break;

	case 0x1:  /* CSI used as system clock */
		sysclk = 4000000U;
		break;

	case 0x2:  /* HSE used as system clock */
		sysclk = HSE_VALUE;
		break;

	case 0x3:  /* PLL1 used as system clock */
		/*
		 * PLL1_P is system clock source
		 * Formula: SYSCLK = HSE / PLLM * PLLN / PLLP
		 */
		pllm = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> 
		       RCC_PLLCKSELR_DIVM1_Pos;
		plln = ((RCC->PLL1DIVR & RCC_PLL1DIVR_N1) >> 
		        RCC_PLL1DIVR_N1_Pos) + 1U;
		pllp = ((RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >> 
		        RCC_PLL1DIVR_P1_Pos) + 1U;

		sysclk = (HSE_VALUE / pllm) * plln / pllp;
		break;

	default:
		sysclk = 64000000U;  /* Default to HSI */
		break;
	}

	SystemCoreClock = sysclk;
}
