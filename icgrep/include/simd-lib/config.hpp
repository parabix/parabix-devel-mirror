#ifndef CONFIG_HPP_
#define CONFIG_HPP_

/*=============================================================================
    config.hpp - Compiler dependent configuration.

    Copyright (C) 2011, Robert D. Cameron, Kenneth S. Herdy, Hua Huang and Nigel Medforth.
    Licensed to the public under the Open Software License 3.0.
    Licensed to International Characters Inc.
       under the Academic Free License version 3.0.
=============================================================================*/

#if !(defined (_MSC_VER) || (defined __GNUC__))
	#error "Compiler not supported. Aborting compilation."
#endif

/*
 * Eliminate definitions of max and min macros from WinDef.h or
 * any other source.   
*/
#undef min
#undef max

/*
 * IDISA_INLINE is a macro which expands to tell the compiler that the method
 * decorated with it should be inlined.  This macro is usable from C and C++
 * code, even though C89 does not support the |inline| keyword.  The compiler
 * may ignore this directive if it chooses.
 */
#ifndef IDISA_INLINE
	#if defined __cplusplus
		#define IDISA_INLINE          inline
	#elif defined _MSC_VER
		#define IDISA_INLINE          __inline
	//	#elif defined __INTEL_COMPILER
	//		#define IDISA_INLINE		  __inline // ICC defaults to GCC. See, Intel® C++ Compiler User and Reference Guides
	#elif defined __GNUC__
		#define IDISA_INLINE          __inline__
	#else
		#define IDISA_INLINE          inline
	#endif
#endif

/*
 * IDISA_INLINE is a macro which expands to tell the compiler that the
 * method decorated with it must be inlined, even if the compiler thinks
 * otherwise.  This is only a (much) stronger version of the IDISA_INLINE hint:
 * compilers are not guaranteed to respect it (although they're much more likely
 * to do so).
 */
#ifndef IDISA_ALWAYS_INLINE
	#if defined IDISA_DEBUG
		#define IDISA_ALWAYS_INLINE   IDISA_INLINE
	#elif defined _MSC_VER
		#define IDISA_ALWAYS_INLINE   __forceinline
	//	#elif defined __INTEL_COMPILER
	//		#define IDISA_ALWAYS_INLINE	__forceinline // ICC defaults to GCC. See, Intel® C++ Compiler User and Reference Guides
	#elif defined __GNUC__
		#define IDISA_ALWAYS_INLINE   __attribute__((always_inline)) IDISA_INLINE
	#else
		#define IDISA_ALWAYS_INLINE   IDISA_INLINE
	#endif
#endif

#endif // CONFIG_HPP_
