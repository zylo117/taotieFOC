/*
*****************************************************************************
**  File        : app_AT32F403AxC_FLASH.ld
**
**  应用链接脚本 —— 和框架默认脚本唯一的功能差别是 FLASH 起点搬到 0x08002000、
**  长度 104K（止于 0x0801C000，校准/滞后角/FastCal 数据区一条线都不能碰），
**  以及 RAM 砍到 95K 给 BL/应用交接页让出最后 1KB。见 include/bl_layout.h。
**
**  原样保留的部分（不要"顺手优化"）：SPIM 段、_sidata、.preinit/.init/.fini_array
**  顺序、DISCARD libc/libm/libgcc —— 这些和现在跑通的镜像逐字对应，改一处就可能
**  让 C++ 全局构造或 .data 装载悄悄坏掉，而且不一定立刻崩。
*****************************************************************************
*/

ENTRY(Reset_Handler)

/* RAM 顶 = 0x20000000 + 95K，交接页在它上面（和 BL 用同一地址） */
_estack = 0x20017C00;

_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
FLASH (rx)      : ORIGIN = 0x08002000, LENGTH = 104K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 95K
SPIM (rx)       : ORIGIN = 0x08400000, LENGTH = 16384K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  _spim_init_base = LOADADDR(.spim);
  _spim_init_length = SIZEOF(.spim);

  .spim :
  {
    . = ALIGN(4);
    _spim_start = .;
    *(.spim)
    *(.spim*)
    . = ALIGN(4);
    _spim_end = .;
  } >SPIM

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
