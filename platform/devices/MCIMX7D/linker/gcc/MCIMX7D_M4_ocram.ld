/*
 Linker file for Colibri i.MX7S and Colibri i.MX7D
 Storing code and data in the TCML / TCMU sections

      __etext +-------------+
              |             |  ^
              |    code     |  |m_text_size
              |             |  v
 m_text_start +-------------+


   __StackTop +-------------+
              |             |  ^  ^
              |    stack    |  |  |STACK_SIZE
              |             |  |  v
 __StackLimit +-------------+  |
  __HeapLimit +-------------+  |
              |             |  |  ^
              |    heap     |  |  |HEAP_SIZE
              |             |  |  v
   __HeapBase +-------------+  |
              |             |  |m_data_size
              | normal data |  |
              |             |  v
 m_data_start +-------------+
*/

TCML_code         = 0x1fff8000;
TCMU_system       = 0x20000000;
OCRAM_code        = 0x00900000;
OCRAM_system      = 0x20200000;
OCRAM_EPDC_code   = 0x00920000;
OCRAM_EPDC_system = 0x20220000;


/* Code execution after reset starts here: */
ENTRY(Reset_Handler)

STACK_SIZE = 0x400;
HEAP_SIZE  = 0x200;
/* Specify the memory areas */
MEMORY
{
  m_text    (RX)  : ORIGIN = OCRAM_EPDC_code, LENGTH = 0x00020000
  m_data    (RW)  : ORIGIN = TCMU_system,     LENGTH = 0x00007FFF
}

SECTIONS
{
  .interrupts :
  {
    __VECTOR_TABLE = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    . = ALIGN(4);
  } > m_text

  __VECTOR_RAM = __VECTOR_TABLE;
  __RAM_VECTOR_TABLE_SIZE_BYTES = 0x0;


  .text :
  {
    . = ALIGN(4);
    *(.text)                 /* code */
    *(.text*)
    *(.rodata)               /* strings and constants */
    *(.rodata*)
    *(.glue_7)               /* thumb <-> arm32 code */
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of the constructors, so we make
       sure it is first.  Because this is a wildcard, it doesn't matter if
       the user does not actually link against crtbegin.o; the linker won't
       look for a file to match a wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from the crtend.o file
       until after the sorted ctors. The .ctor section from the crtend file
       contains the end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_text

  __etext = .;              /* create a global symbol at end of code */
  __DATA_ROM = .;           /* Used by startup for data initialization */

  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    *(.data)                 /* regular data */
    *(.data*)
    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > m_data

  /* Heap follows right behind the regular data */
  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    . += HEAP_SIZE;
    __HeapLimit = .;
  } > m_data
  .stack :
  {
    . = ALIGN(8);
    . += STACK_SIZE;
  } > m_data
  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(m_data) + LENGTH(m_data);
  __StackLimit = __StackTop - STACK_SIZE;
  PROVIDE(__stack = __StackTop);
  .ARM.attributes 0 : { *(.ARM.attributes) }

  ASSERT(__StackLimit >= __HeapLimit, "stack and heap overflow")
 }



