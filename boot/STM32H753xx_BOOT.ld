/*
******************************************************************************
**  The bootloader's linker script: sector 0 of bank 1, and everything
**  copied out of it at reset (docs/BOOT.md).
**
**  The vector table and the reset handler stay in flash - they run before
**  anything is copied. .text and .rodata load from flash and run from
**  ITCM; .data loads from flash and lives in DTCM; so once main() runs
**  the core never fetches from flash, and erasing or programming a bank
**  stalls nothing it does. The handover slot is the top 32 bytes of
**  DTCM, the same address the application's script gives it, NOLOAD and
**  outside .bss so neither startup touches it.
******************************************************************************
*/

ENTRY(Reset_Handler)

MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH (rx)         : ORIGIN = 0x08000000, LENGTH = 128K
}

_Boot_Hand_Size = 32;
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM) - _Boot_Hand_Size;
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x800;

SECTIONS
{
  /* What runs from flash: the vectors and the copy loops. */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    *(.text.Reset_Handler)
    *(.text.Default_Handler)
    . = ALIGN(4);
  } >FLASH

  /* Code and constants, copied to ITCM by the startup. */
  _sitext = LOADADDR(.text);
  .text :
  {
    . = ALIGN(4);
    _stext = .;
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    *(.rodata)
    *(.rodata*)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >ITCMRAM AT> FLASH

  /* Initialised data, copied to DTCM by the startup. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCMRAM AT> FLASH

  .bss (NOLOAD) :
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
  } >DTCMRAM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  .boot_hand _estack (NOLOAD) :
  {
    KEEP(*(.boot_hand))
  } >DTCMRAM

  /* Nothing here unwinds: the tables libgcc's helpers carry are dropped. */
  /DISCARD/ :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    *(.ARM.exidx*)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
