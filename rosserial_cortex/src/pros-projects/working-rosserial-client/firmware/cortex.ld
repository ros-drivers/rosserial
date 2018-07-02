/* Include STM32F10X linker scripts */
INCLUDE "STM32F10x.ld"

EXTERN ( _heapbegin );

/* Section definitions for Cortex firmware flashing */
SECTIONS {
	.isr_vector : {
		/* startup code, prevents garbage collection from eating everything */
		KEEP(*(.isr_vector))
	. = ALIGN(4);
	} >FLASH
	.text : {
	. = ALIGN(4);
		*(.text)
		*(.text.*)
		/* Preinit array of functions */
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
		. = ALIGN(4);
		/* Init array of functions */
		KEEP(*(.init))
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
		. = ALIGN(4);
		/* C++ constructors */
		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))
		. = ALIGN(4);
		/* Finalizer array of functions */
		KEEP(*(.fini))
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
		. = ALIGN(4);
		/* C++ destructors */
		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))
		_etext = .;
		_srdata = .;
	. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)
	. = ALIGN(4);
		_erdata = .;
		_sidata = .;
	} >FLASH
	/* The program executes knowing that the data is in the RAM, but the loader puts the
	 * initial values in the FLASH (inidata). The startup copies the initial values over */
	.data : AT ( _sidata ) {
	. = ALIGN(4);
		_sdata = .;
		*(.data)
		*(.data.*)
		*(.RAMtext)
	. = ALIGN(4);
   		_edata = .;
	} >RAM
	/* Uninitialized data (zero-fill) section */
	.bss : {
	. = ALIGN(4);
		_sbss = .;
		*(.bss)
		*(COMMON)
	. = ALIGN(4);
		_ebss = .;
	. = ALIGN(8);
		_heapbegin = .;
	} >RAM

	/DISCARD/ : {
		libc.a ( * )
		libg.a ( * )
		libm.a ( * )
		libgcc.a ( * )
		libstdc++.a ( * )
		libsupc++.a ( * )
	}
	/* ARM exception unwinding, mandated by ARM EABI C++ standard (with -fno-exceptions?) */
	.ARM.exidx 0 : { *(.ARM.exidx*) }
	/* Stabs debugging sections */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index	0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	/* DWARF 1 */
	.debug 0 : { *(.debug) }
	.line 0 : { *(.line) }
	/* GNU DWARF 1 extensions */
	.debug_srcinfo 0 : { *(.debug_srcinfo) }
	.debug_sfnames 0 : { *(.debug_sfnames) }
	/* DWARF 1.1 and DWARF 2 */
	.debug_aranges 0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	/* DWARF 2 */
	.debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_line 0 : { *(.debug_line) }
	.debug_frame 0 : { *(.debug_frame) }
	.debug_str 0 : { *(.debug_str) }
	.debug_loc 0 : { *(.debug_loc) }
	.debug_macinfo 0 : { *(.debug_macinfo) }
	/* SGI/MIPS DWARF 2 extensions */
	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames 0 : { *(.debug_varnames) }
}
