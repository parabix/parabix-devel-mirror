#ifndef BITBLOCK_HPP
#define BITBLOCK_HPP

/*=============================================================================
	Copyright (C) 2011, Robert D. Cameron, Kenneth S. Herdy
	Licensed to the public under the Open Software License 3.0.
	Licensed to International Characters Inc.
	   under the Academic Free License version 3.0.
=============================================================================*/

// #define NDEBUG // if NDEBUG then disable assertions

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#include "config.hpp"
#include "builtins.hpp"
#include "idisa.hpp"

#define BytePack BitBlock
#ifndef BLOCK_SIZE
#define BLOCK_SIZE 128
#endif
#ifndef ATTRIBUTE_SIMD_ALIGN
	#if defined _MSC_VER
		//note: MSVC++ cannot accept sizeof or division within __declspec(align(...))
		#define ATTRIBUTE_SIMD_ALIGN __declspec(align(16))
	#elif defined __GNUC__
		#define ATTRIBUTE_SIMD_ALIGN __attribute__((aligned(sizeof(BitBlock))))
	#else
		#define ATTRIBUTE_SIMD_ALIGN
	#endif
#endif

template<class T> void print_register(const char * var_name, T v);

static IDISA_ALWAYS_INLINE uint32_t count_forward_zeroes(BitBlock v);
static IDISA_ALWAYS_INLINE uint32_t count_reverse_zeroes(BitBlock v);
static IDISA_ALWAYS_INLINE BitBlock mask_forward_zeroes(uint32_t count);
static IDISA_ALWAYS_INLINE BitBlock mask_reverse_zeroes(uint32_t count);
static IDISA_ALWAYS_INLINE uint32_t bitstream_scan(BitBlock * v, uint32_t pos);

/*  BitBlock union type */
union ubitblock;

/*  Default BLOCK_SIZE is 128, compatible with SSE, Altivec, SPU */
#if (BLOCK_SIZE == 128)
#include "bitblock128.hpp"
#endif

/*  BLOCK_SIZE 256 for AVX */
#if (BLOCK_SIZE == 256)
#include "bitblock256.hpp"
#endif

template <class T> void print_register(const char * var_name, T v);
template <class T>
void print_register(const char * var_name, T v) {
	unsigned char c;
	printf("%40s = ", var_name);
	for(int i=sizeof(T)-1; i>=0; i--) {
		c = *(((unsigned char *)&v)+i);
		printf("%02X ", c);
	}
	printf("\n");
}

IDISA_ALWAYS_INLINE uint32_t count_forward_zeroes(BitBlock v) {
	union {BitBlock bitblock; ScanWord elems[sizeof(BitBlock)/sizeof(ScanWord)];} u;
	u.bitblock = v;
	uint32_t so_far = 0;
	for (unsigned int i = 0; i < sizeof(BitBlock)/sizeof(ScanWord); i++) {
		if (u.elems[i] != 0) return so_far | scan_forward_zeroes(u.elems[i]);
		so_far += 8 * sizeof(ScanWord);
	}
	return so_far;
}

IDISA_ALWAYS_INLINE BitBlock mask_forward_zeroes(uint32_t count) {
	if (count >= sizeof(BitBlock) * 8) return simd<1>::constant<0>();
		else return bitblock::sll(simd<1>::constant<1>(), convert(count));
}

IDISA_ALWAYS_INLINE uint32_t count_reverse_zeroes(BitBlock v) {
	union {BitBlock bitblock; ScanWord elems[sizeof(BitBlock)/sizeof(ScanWord)];} u;
	u.bitblock = v;
	uint32_t so_far = 0;
	for (unsigned int i = (sizeof(BitBlock)/sizeof(ScanWord)); i != 0; ) {
		if (u.elems[--i] != 0) return so_far | scan_backward_zeroes(u.elems[i]);
		so_far += 8 * sizeof(ScanWord);
	}
	return so_far;
}

IDISA_ALWAYS_INLINE BitBlock mask_reverse_zeroes(uint32_t count) {
	if (count >= sizeof(BitBlock) * 8) return simd<1>::constant<0>();
		else return bitblock::srl(simd<1>::constant<1>(), convert(count));
}

IDISA_ALWAYS_INLINE uint32_t bitstream_scan(BitBlock * v, uint32_t pos) {
	ScanWord * bitstream_ptr = (ScanWord *) (((intptr_t) v) + pos/8);
	ScanWord bitstream_slice = ((*bitstream_ptr) >> (pos % 8));
	if (bitstream_slice != 0) return pos + scan_forward_zeroes(bitstream_slice);
	else {
		do {
			bitstream_ptr++;
			bitstream_slice = *bitstream_ptr;
		} while (bitstream_slice == 0);
		uint32_t base_posn = 8*((intptr_t) bitstream_ptr - (intptr_t) v);
		return base_posn + scan_forward_zeroes(bitstream_slice);
	}
}

static IDISA_ALWAYS_INLINE void assert_bitblock_align(void * addr) {
	assert(0 == ((intptr_t)(addr) & (sizeof(BitBlock)-1)));
}

static IDISA_ALWAYS_INLINE void assert_bitblock_align(BitBlock v) {
	assert(0 == ((intptr_t)(&v) & (sizeof(BitBlock)-1)));
}

#define ASSERT_BITBLOCK_ALIGN(v) assert_bitblock_align(v)

#endif // BITBLOCK_HPP


