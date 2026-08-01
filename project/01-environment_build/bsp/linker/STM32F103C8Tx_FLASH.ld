/*
 * STM32F103C8Tx Linker Script
 * Flash: 64KB @ 0x08000000
 * SRAM:  20KB @ 0x20000000
 */
/*
 * 链接脚本的职责：
 * 1.定义flash与ram的大小与起始地址
 * 2.定义代码段与数据段的地址（.text,.data,.bss）
 * 3.提供符号给启动脚本使用（_estack，_Min_Heap_Size，_Min_Stack_Size等）
 * 4.预留栈空间
 */
MEMORY
{
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
}

/* 栈顶 = RAM 末尾 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 最小堆栈保留 */
_Min_Heap_Size  = 0x200;  /* 512B heap  */
_Min_Stack_Size = 0x400;  /* 1KB stack  */

ENTRY(Reset_Handler)

SECTIONS
{
  /* 中断向量表 - 必须放在 FLASH 最前面 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* 代码段 */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7t)
    *(.glue_7)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* 只读数据段 */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* ARM 特殊段 */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* 已初始化数据段 - 存储在 FLASH，运行时拷贝到 RAM */
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

  /* 未初始化数据段 (BSS) - 运行时清零 */
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
  } >RAM

  /* 用户堆栈校验 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* 丢弃调试信息以外的无用段 */
  /DISCARD/ :
  {
    libc.a(*)
    libm.a(*)
    libgcc.a(*)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}