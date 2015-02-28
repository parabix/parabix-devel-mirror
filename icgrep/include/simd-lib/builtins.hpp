#ifndef BUILTINS_HPP_
#define BUILTINS_HPP_

/*=============================================================================
    builtin - Compiler dependent builtin function wrappers.

    Copyright (C) 2011, Robert D. Cameron, Kenneth S. Herdy.
    Licensed to the public under the Open Software License 3.0.
    Licensed to International Characters Inc.
       under the Academic Free License version 3.0. 
=============================================================================*/

#include "config.hpp"

static IDISA_ALWAYS_INLINE long likely(long x);
static IDISA_ALWAYS_INLINE long unlikely(long x);

#if defined (_MSC_VER)
	IDISA_ALWAYS_INLINE long likely(long x) {
		return x;
	}
	IDISA_ALWAYS_INLINE long unlikely(long x) {
		return x;
	}

#elif defined (__GNUC__)

	IDISA_ALWAYS_INLINE long likely(long x) {
		return __builtin_expect(x, 1);
	}

	IDISA_ALWAYS_INLINE long unlikely(long x) {
		return __builtin_expect(x, 0);
	}

#endif

#include <stdint.h>
#if defined (_MSC_VER)
        #include <intrin.h>

        #ifdef _M_X64
        typedef unsigned __int64 ScanWord;
        #define ScanForwardIntrinsic _BitScanForward64
        #define ScanReverseIntrinsic _BitScanReverse64
        #else
        typedef unsigned __int32 ScanWord;
        #define ScanForwardIntrinsic _BitScanForward
        #define ScanReverseIntrinsic _BitScanReverse
        #endif

        #pragma intrinsic(ScanForwardIntrinsic)
        #pragma intrinsic(ScanReverseIntrinsic)
        IDISA_ALWAYS_INLINE uint32_t scan_forward_zeroes(ScanWord x) { // Precondition: x != 0
                unsigned long zeroes;
                ScanForwardIntrinsic(&zeroes, x);
                return (uint32_t) zeroes;
        }

        IDISA_ALWAYS_INLINE uint32_t scan_backward_zeroes(ScanWord x) { // Precondition: x != 0
                unsigned long pos;
                ScanReverseIntrinsic(&pos, x);
                return (uint32_t) (8* sizeof(ScanWord) - pos - 1);
        }

	#undef ScanForwardIntrinsic
	#undef ScanReverseIntrinsic

#elif defined (__GNUC__)

        #if __x86_64__ /* 64-bit architecture */
        typedef uint64_t ScanWord;
        #define ScanForwardIntrinsic __builtin_ctzll
        #define ScanReverseIntrinsic __builtin_clzll

        #else
        typedef uint32_t ScanWord;
        #define ScanForwardIntrinsic __builtin_ctzl
        #define ScanReverseIntrinsic __builtin_clzl
        #endif

        IDISA_ALWAYS_INLINE int scan_forward_zeroes(ScanWord x) { // Precondition: x != 0
                return ScanForwardIntrinsic((ScanWord) x);
        }

        IDISA_ALWAYS_INLINE int scan_backward_zeroes(ScanWord x) { // Precondition: x != 0
		return ScanReverseIntrinsic((ScanWord) x);
        }

	#undef ScanForwardIntrinsic
	#undef ScanReverseIntrinsic

#else
  #warning "Neither _MSC_VER nor __GNUC__ defined: scan_forward/backward_zeroes not implemented."
#endif

#endif // BUILTINS_HPP
