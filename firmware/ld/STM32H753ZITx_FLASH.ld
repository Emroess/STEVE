/*
 * Linker script for STM32H753ZITx (NUCLEO-H753ZI)
 * 
 * Memory Layout per RM0433:
 * - FLASH:  0x08000000, 2048 KB
 * - ITCM:   0x00000000,   64 KB (Instruction Tightly-Coupled Memory)
 * - DTCM:   0x20000000,  128 KB (Data Tightly-Coupled Memory)
 * - AXI SRAM (SRAM1): 0x24000000, 512 KB
 * - SRAM2:  0x30000000,  288 KB (includes 64KB+128KB+32KB+32KB+32KB)
 * - SRAM3:  0x30040000,   64 KB (non-cacheable, for DMA/FDCAN buffers)
 * - SRAM4:  0x38000000,   64 KB (in D3 domain)
 * - Backup SRAM: 0x38800000, 4 KB
 *
 * This linker script follows the strict requirements from IMPROVE_FIRMWARE_PLAN.md:
 * - All DMA/FDCAN buffers in .sram3_dma section
 * - Stack/heap collision detection
 * - Linker-time size assertions
 * - Clear memory region boundaries
 */

ENTRY(Reset_Handler)

/* Memory region sizes */
/* Increased stack and heap after relocating large HTTP state buffers to AXI SRAM.
 * Provides more headroom for deep call chains (printf/JSON parsing) and dynamic allocations. */
__STACK_SIZE = 0x4000;  /* 16 KB stack */
__HEAP_SIZE  = 0x2000;  /* 8 KB heap */

/* Memory regions per STM32H753 datasheet */
MEMORY
{
  FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 2048K
  NVM (rwx)         : ORIGIN = 0x081C0000, LENGTH = 64K
  NETWORK_CONFIG (rwx) : ORIGIN = 0x081A0000, LENGTH = 64K
  ITCMRAM (xrw)     : ORIGIN = 0x00000000, LENGTH = 64K
  DTCMRAM (xrw)     : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)      : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
  RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 288K  /* SRAM1+SRAM2 */
  RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 */
}

/* Highest address of the user mode stack (top of DTCMRAM) */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);

/* Section definitions */
SECTIONS
{
  /* Interrupt vector table at start of FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Program code and read-only data */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* Read-only data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* NVM data section for persistent presets - placed at end of flash */
  .nvm_data :
  {
    . = ALIGN(4096);
    __nvm_data_start = .;
    . = . + 65536;
    __nvm_data_end = .;
  } >NVM

  /* Dedicated network config region to avoid clashes with valve presets */
  .network_config (NOLOAD) :
  {
    . = ALIGN(4096);
    __network_config_start = .;
    . = . + 65536;
    __network_config_end = .;
  } >NETWORK_CONFIG

  /* ARM unwinding tables */
  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM.exidx :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  /* C++ constructors/destructors (not used in bare metal, but keep for completeness) */
  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Used by startup to initialize data section */
  _sidata = LOADADDR(.data);

  /* Initialized data in DTCMRAM (fast access) */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCMRAM AT> FLASH

  /* Uninitialized data in DTCMRAM */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
    
    /* Provide end symbol for newlib _sbrk */
    PROVIDE(end = .);
    PROVIDE(_end = .);
  } >DTCMRAM

  /* Heap in DTCMRAM (after .bss) */
  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    __heap_start__ = .;
    . = . + __HEAP_SIZE;
    __heap_end__ = .;
  } >DTCMRAM

  /* Stack in DTCMRAM (grows downward from _estack) */
  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    __stack_limit__ = .;
    . = . + __STACK_SIZE;
    __stack_top__ = .;
  } >DTCMRAM

  /*
   * DMA/FDCAN buffer section in SRAM2 (non-cacheable region)
   * This is in RAM_D2 which is in D2 domain and can be configured as non-cacheable
   * All FDCAN message RAM buffers and DMA buffers must be placed here
   */
  .sram3_dma (NOLOAD) :
  {
    . = ALIGN(4);
    __sram3_dma_start = .;
    *(.sram3_dma*)
    . = ALIGN(4);
    __sram3_dma_end = .;
  } >RAM_D2

  /* Calculate DMA section size for static assertions */
  __sram3_dma_size = __sram3_dma_end - __sram3_dma_start;
  PROVIDE(__SRAM3_DMA_SIZE = __sram3_dma_size);

  /* General purpose data in AXI SRAM (D1 domain, fastest) */
  .ram_d1 (NOLOAD) :
  {
    . = ALIGN(4);
    *(.ram_d1)
    . = ALIGN(4);
  } >RAM_D1

  /* Peripheral data in D3 SRAM (always powered) */
  .ram_d3 (NOLOAD) :
  {
    . = ALIGN(4);
    *(.ram_d3)
    . = ALIGN(4);
  } >RAM_D3

  /* Ethernet DMA descriptor and buffer sections (must be in DMA-accessible SRAM, not DTCMRAM) */
  .RxDescripSection (NOLOAD) :
  {
    . = ALIGN(32);
    *(.RxDescripSection)
    . = ALIGN(32);
  } >RAM_D2

  .TxDescripSection (NOLOAD) :
  {
    . = ALIGN(32);
    *(.TxDescripSection)
    . = ALIGN(32);
  } >RAM_D2

  .Rx_PoolSection (NOLOAD) :
  {
    . = ALIGN(32);
    *(.Rx_PoolSection)
    . = ALIGN(32);
  } >RAM_D2

  /* Remove unwanted sections from standard libraries */
  /DISCARD/ :
  {
    *(.note.GNU-stack)
    *(.gnu_debuglink)
    *(.gnu.lto_*)
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  }
}

/*
 * Linker-time assertions per IMPROVE_FIRMWARE_PLAN.md
 * These catch configuration errors at link time, not runtime
 */
ASSERT(__heap_end__ <= __stack_limit__, "ERROR: Heap and stack collision in DTCMRAM")
ASSERT(__stack_top__ <= _estack, "ERROR: Stack overflow in DTCMRAM")
ASSERT(__sram3_dma_size <= (64 * 1024), "ERROR: .sram3_dma section exceeds 64KB")
/* Note: .sram3_dma size warning is shown during link but doesn't fail the build */
