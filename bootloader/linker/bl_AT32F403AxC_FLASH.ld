/*
*****************************************************************************
**  File        : bl_AT32F403AxC_FLASH.ld
**
**  Bootloader 閾炬帴鑴氭湰 鈥斺€?Flash 鍙粰 8KB锛?x08000000 ~ 0x08001FFF锛夈€?**  RAM 鐮嶅埌 95KB锛屾渶鍚?1KB锛?x20017C00锛夌暀缁?BL/搴旂敤浜ゆ帴椤碉紙瑙?include/bl_layout.h锛夛細
**  瀹冨繀椤昏惤鍦?.bss/.data/鍫嗘爤涔嬪锛屾墠涓嶄細琚惎鍔ㄤ唬鐮佹竻闆躲€佷篃涓嶄細琚爤韪╁埌銆?**  8KB 鏄繖涓?BL 鐨勭‖棰勭畻锛氬簲鐢ㄧ幇鍦?99,720 瀛楄妭锛孊L 鍐嶅ぇ搴旂敤灏辨病鍦版柟闀夸簡銆?*****************************************************************************
*/

ENTRY(Reset_Handler)

/* RAM 椤?= 0x20000000 + 95K锛屼氦鎺ラ〉鍦ㄥ畠涓婇潰 */
_estack = 0x20017C00;

_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x800;    /* BL 閲屾湁 flash 鎿﹀啓 + USB 鎺у埗浼犺緭锛屾爤缁欒冻 2KB */

MEMORY
{
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 8K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 95K
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

  .preinit_array :
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

  /* 浜ゆ帴椤?0x20017C00 鏁呮剰涓嶅嚭鐜板湪浠讳綍娈甸噷锛歊AM 鍙０鏄庡埌 95K锛岄摼鎺ュ櫒涓嶄細鐢ㄥ埌瀹?*/

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
