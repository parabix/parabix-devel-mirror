/*
  Copyright (c) 2010-2016, Intel Corporation
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.


   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
   IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
   PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** @file ispc.cpp
    @brief ispc global definitions
*/

#include "ispc.h"
#include <stdexcept>

///////////////////////////////////////////////////////////////////////////
// Target

#if !defined(ISPC_IS_WINDOWS) && !defined(__arm__)
#include <cpuid.h> // supplied by clang and gcc
static inline void cpuid(int info[4], int level) {
    __cpuid(level, info[0], info[1], info[2], info[3]);
}

static void cpuid_count(int info[4], int level, int count) {
    __cpuid_count(level, count, info[0], info[1], info[2], info[3]);
}
#else
static inline void cpuid(int info[4], int level) {
    __cpuid(info, level);
}

static void cpuid_count(int info[4], int level, int count) {
    __cpuidex(info, level, count);
}
#endif // !ISPC_IS_WINDOWS && !__ARM__

#if !defined(__arm__)
bool __os_has_avx_support() {
#if defined(ISPC_IS_WINDOWS)
    // Check if the OS will save the YMM registers
    unsigned long long xcrFeatureMask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
    return (xcrFeatureMask & 6) == 6;
#else // !defined(ISPC_IS_WINDOWS)
    // Check xgetbv; this uses a .byte sequence instead of the instruction
    // directly because older assemblers do not include support for xgetbv and
    // there is no easy way to conditionally compile based on the assembler used.
    int rEAX, rEDX;
    __asm__ __volatile__ (".byte 0x0f, 0x01, 0xd0" : "=a" (rEAX), "=d" (rEDX) : "c" (0));
    return (rEAX & 6) == 6;
#endif // !defined(ISPC_IS_WINDOWS)
}

bool __os_has_avx512_support() {
#if defined(ISPC_IS_WINDOWS)
    // Check if the OS saves the XMM, YMM and ZMM registers, i.e. it supports AVX2 and AVX512.
    // See section 2.1 of software.intel.com/sites/default/files/managed/0d/53/319433-022.pdf
    unsigned long long xcrFeatureMask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
    return (xcrFeatureMask & 0xE6) == 0xE6;
#else // !defined(ISPC_IS_WINDOWS)
    // Check xgetbv; this uses a .byte sequence instead of the instruction
    // directly because older assemblers do not include support for xgetbv and
    // there is no easy way to conditionally compile based on the assembler used.
    int rEAX, rEDX;
    __asm__ __volatile__ (".byte 0x0f, 0x01, 0xd0" : "=a" (rEAX), "=d" (rEDX) : "c" (0));
    return (rEAX & 0xE6) == 0xE6;
#endif // !defined(ISPC_IS_WINDOWS)
}
#endif // !__arm__

const char *
lGetSystemISA() {
#ifdef __arm__
    return "neon-i32x4";
#else
    int info[4];
    cpuid(info, 1);

    int info2[4];
    // Call cpuid with eax=7, ecx=0
    cpuid_count(info2, 7, 0);

    if ((info[2] & (1 << 27)) != 0 &&  // OSXSAVE
        (info2[1] & (1 <<  5)) != 0 && // AVX2
        (info2[1] & (1 << 16)) != 0 && // AVX512 F
        __os_has_avx512_support()) {
        // We need to verify that AVX2 is also available,
        // as well as AVX512, because our targets are supposed
        // to use both.

        if ((info2[1] & (1 << 17)) != 0 && // AVX512 DQ
            (info2[1] & (1 << 28)) != 0 && // AVX512 CDI
            (info2[1] & (1 << 30)) != 0 && // AVX512 BW
            (info2[1] & (1 << 31)) != 0) { // AVX512 VL
            return "avx512skx-i32x16";
        }
        else if ((info2[1] & (1 << 26)) != 0 && // AVX512 PF
                 (info2[1] & (1 << 27)) != 0 && // AVX512 ER
                 (info2[1] & (1 << 28)) != 0) { // AVX512 CDI
            return "avx512knl-i32x16";
        }
        // If it's unknown AVX512 target, fall through and use AVX2
        // or whatever is available in the machine.
    }

    if ((info[2] & (1 << 27)) != 0 && // OSXSAVE
        (info[2] & (1 << 28)) != 0 &&
         __os_has_avx_support()) {  // AVX
        // AVX1 for sure....
        // Ivy Bridge?
        if ((info[2] & (1 << 29)) != 0 &&  // F16C
            (info[2] & (1 << 30)) != 0) {  // RDRAND
            // So far, so good.  AVX2?
            if ((info2[1] & (1 << 5)) != 0)
                return "avx2-i32x8";
            else
                return "avx1.1-i32x8";
        }
        // Regular AVX
        return "avx1-i32x8";
    }
    else if ((info[2] & (1 << 19)) != 0)
        return "sse4-i32x4";
    else if ((info[3] & (1 << 26)) != 0)
        return "sse2-i32x4";
    else {
        throw std::runtime_error("Unable to detect supported SSE/AVX ISA.");
    }
#endif
}


