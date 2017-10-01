#ifndef _LIB_MISC_H_
#define _LIB_MISC_H_

#include <avr/pgmspace.h>

/*
 * source:
 * http://michael-buschbeck.github.io/arduino/2013/10/22/string-merging-pstr-percent-codes/
 */
#define _PSTR_M_STRINGIFY(str) #str
#define PSTR_M(str)							\
	(__extension__({						\
			PGM_P ptr;					\
									\
			asm volatile (					\
				      ".pushsection .progmem.data, \"SM\", @progbits, 1" "\n\t" \
				      "0: .string " _PSTR_M_STRINGIFY(str)               "\n\t" \
				      ".popsection"                                      "\n\t" \
				      );				\
									\
			asm volatile (					\
				      "ldi %A0, lo8(0b)"                                 "\n\t" \
				      "ldi %B0, hi8(0b)"                                 "\n\t" \
				      : "=d" (ptr)			\
				      );				\
									\
			ptr;						\
			})						\
	 )

#endif
