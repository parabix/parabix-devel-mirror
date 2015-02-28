#ifndef BITBLOCK256_HPP_
#define BITBLOCK256_HPP_

/*=============================================================================
    bitblock256 - Specific 256 bit implementations.

    Copyright (C) 2011, Robert D. Cameron, Kenneth S. Herdy, Hua Huang and Nigel Medforth.
    Licensed to the public under the Open Software License 3.0.
    Licensed to International Characters Inc.
       under the Academic Free License version 3.0.

=============================================================================*/

#include "idisa128.hpp"
#include "idisa256.hpp"
#include "builtins.hpp"

union ubitblock {
        bitblock256_t _bitblock;
        bitblock256_t _256;
        bitblock128_t _128[sizeof(bitblock256_t)/sizeof(bitblock256_t)];
        uint64_t _64[sizeof(bitblock256_t)/sizeof(uint64_t)];
        uint32_t _32[sizeof(bitblock256_t)/sizeof(uint32_t)];
        uint16_t _16[sizeof(bitblock256_t)/sizeof(uint16_t)];
        uint8_t _8[sizeof(bitblock256_t)/sizeof(uint8_t)];
};

/* The type used to store a carry bit. */
#ifndef CARRY64
typedef bitblock256_t carry_t;
#endif
#ifdef CARRY64
typedef uint64_t carry_t;
#endif

static IDISA_ALWAYS_INLINE void add_ci_co(bitblock256_t x, bitblock256_t y, carry_t carry_in, carry_t & carry_out, bitblock256_t & sum);
static IDISA_ALWAYS_INLINE void sub_bi_bo(bitblock256_t x, bitblock256_t y, carry_t borrow_in, carry_t & borrow_out, bitblock256_t & difference);
static IDISA_ALWAYS_INLINE void adv_ci_co(bitblock256_t cursor, carry_t carry_in, carry_t & carry_out, bitblock256_t & rslt);




static IDISA_ALWAYS_INLINE bitblock256_t carry2bitblock(carry_t carry);
static IDISA_ALWAYS_INLINE carry_t bitblock2carry(bitblock256_t carry);
static IDISA_ALWAYS_INLINE carry_t carryout2carry(bitblock256_t carryout);


static IDISA_ALWAYS_INLINE void adc(bitblock256_t x, bitblock256_t y, carry_t & carry, bitblock256_t & sum);
static IDISA_ALWAYS_INLINE void sbb(bitblock256_t x, bitblock256_t y, carry_t & borrow, bitblock256_t & difference);
static IDISA_ALWAYS_INLINE void advance_with_carry(bitblock256_t cursor, carry_t & carry, bitblock256_t & rslt);

static IDISA_ALWAYS_INLINE void adc(bitblock256_t x, bitblock256_t y, carry_t carry_in, carry_t & carry_out, bitblock256_t & sum);
static IDISA_ALWAYS_INLINE void sbb(bitblock256_t x, bitblock256_t y, carry_t borrow_in, carry_t & borrow_out, bitblock256_t & difference);
static IDISA_ALWAYS_INLINE void advance_with_carry(bitblock256_t cursor, carry_t carry_in, carry_t & carry_out, bitblock256_t & rslt);

static IDISA_ALWAYS_INLINE bitblock256_t convert (uint64_t s);
static IDISA_ALWAYS_INLINE bitblock128_t convert_128 (uint64_t s);
static IDISA_ALWAYS_INLINE uint64_t convert (bitblock256_t v);

#ifndef CARRY64
static IDISA_ALWAYS_INLINE bitblock256_t carry2bitblock(carry_t carry) {  return carry;}
static IDISA_ALWAYS_INLINE carry_t bitblock2carry(bitblock256_t carry) {  return carry;}
static IDISA_ALWAYS_INLINE uint64_t carry2uint64(carry_t carry) {  return convert(carry);}
static IDISA_ALWAYS_INLINE carry_t uint2carry(uint64_t carry) {  return convert(carry);}
#endif
#ifdef CARRY64
static IDISA_ALWAYS_INLINE bitblock256_t carry2bitblock(carry_t carry) {  return convert(carry);}
static IDISA_ALWAYS_INLINE carry_t bitblock2carry(bitblock256_t carry) {  return convert(carry);}
static IDISA_ALWAYS_INLINE uint64_t carry2uint64(carry_t carry) {  return carry;}
static IDISA_ALWAYS_INLINE carry_t uint2carry(uint64_t carry) {  return carry;}
#endif

static IDISA_ALWAYS_INLINE carry_t carryout2carry(carry_t carryout) {
  return carryout;
}

static IDISA_ALWAYS_INLINE void add_ci_co(bitblock256_t x, bitblock256_t y, carry_t carry_in, carry_t & carry_out, bitblock256_t & sum) {
  bitblock256_t all_ones = simd256<1>::constant<1>();
  bitblock256_t gen = simd_and(x, y);
  bitblock256_t prop = simd_xor(x, y);
  bitblock256_t partial_sum = simd256<64>::add(x, y);
  bitblock256_t carry = simd_or(gen, simd_andc(prop, partial_sum));
  bitblock256_t bubble = simd256<64>::eq(partial_sum, all_ones);
  uint64_t carry_mask = hsimd256<64>::signmask(carry) * 2 + carry2uint64(carry_in);
  uint64_t bubble_mask = hsimd256<64>::signmask(bubble);
  uint64_t carry_scan_thru_bubbles = (carry_mask + bubble_mask) &~ bubble_mask;
  uint64_t increments = carry_scan_thru_bubbles | (carry_scan_thru_bubbles - carry_mask);
  carry_out = uint2carry(increments >> 4);
  uint64_t spread = 0x0000200040008001 * increments & 0x0001000100010001;
  sum = simd256<64>::add(partial_sum, _mm256_cvtepu16_epi64(avx_select_lo128(convert(spread))));
}

static IDISA_ALWAYS_INLINE void sub_bi_bo(bitblock256_t x, bitblock256_t y, carry_t borrow_in, carry_t & borrow_out, bitblock256_t & difference){
  bitblock256_t gen = simd_andc(y, x);
  bitblock256_t prop = simd_not(simd_xor(x, y));
  bitblock256_t partial_diff = simd256<64>::sub(x, y);
  bitblock256_t borrow = simd_or(gen, simd_and(prop, partial_diff));
  bitblock256_t bubble = simd256<64>::eq(partial_diff, simd<1>::constant<0>());
  uint64_t borrow_mask = hsimd256<64>::signmask(borrow) * 2 + carry2uint64(borrow_in);
  uint64_t bubble_mask = hsimd256<64>::signmask(bubble);
  uint64_t borrow_scan_thru_bubbles = (borrow_mask + bubble_mask) &~ bubble_mask;
  uint64_t decrements = borrow_scan_thru_bubbles | (borrow_scan_thru_bubbles - borrow_mask);
  borrow_out = uint2carry(decrements >> 4);
  uint64_t spread = 0x0000200040008001 * decrements & 0x0001000100010001;
  difference = simd256<64>::sub(partial_diff, _mm256_cvtepu16_epi64(avx_select_lo128(convert(spread))));
}

static IDISA_ALWAYS_INLINE void adv_ci_co(bitblock256_t cursor, carry_t carry_in, carry_t & carry_out, bitblock256_t & rslt){
	bitblock256_t shift_out = simd256<64>::srli<63>(cursor);
	bitblock256_t low_bits = simd_or(mvmd256<64>::slli<1>(shift_out), carry2bitblock(carry_in));
	carry_out = bitblock2carry(mvmd256<64>::srli<3>(shift_out));
	rslt = simd_or(simd256<64>::add(cursor, cursor), low_bits);
}

IDISA_ALWAYS_INLINE bitblock256_t convert(uint64_t s)
{  
  return _mm256_castsi128_si256(_mm_cvtsi64_si128(s));
  // ubitblock b = {b._256 = simd256<128>::constant<0>()}; // = {0};
  // b._64[0] = s;
  // return b._256;
}

IDISA_ALWAYS_INLINE bitblock128_t convert_128(uint64_t s)
{
  ubitblock b = {b._256 = simd256<128>::constant<0>()}; // = {0};
  b._64[0] = s;
  return b._128[0];
}

IDISA_ALWAYS_INLINE uint64_t convert(bitblock256_t v)
{
  return (uint64_t) _mm_cvtsi128_si64(avx_select_lo128(v));
  // return (uint64_t) mvmd256<64>::extract<0>(v);
}

// The code below is not used.

#ifdef AVX
#define avx_select_lo128(x) \
	((__m128i) _mm256_castps256_ps128(x))

#define avx_select_hi128(x) \
	((__m128i)(_mm256_extractf128_ps(x, 1)))

#define avx_general_combine256(x, y) \
   (_mm256_insertf128_ps(_mm256_castps128_ps256((__m128) y), (__m128) x, 1))

IDISA_ALWAYS_INLINE void adc128(bitblock128_t x, bitblock128_t y, bitblock128_t carry_in, bitblock128_t & carry_out, bitblock128_t & sum)
{
	bitblock128_t gen = simd_and(x, y);
	bitblock128_t prop = simd_or(x, y);
	bitblock128_t partial = simd128<64>::add(simd128<64>::add(x, y), carry_in);
	bitblock128_t c1 = simd128<128>::slli<64>(simd128<64>::srli<63>(simd_or(gen, simd_andc(prop, partial))));
	sum = simd128<64>::add(c1, partial);
	carry_out = simd128<128>::srli<127>(simd_or(gen, simd_andc(prop, sum)));
}

#ifndef ADCMAGIC
IDISA_ALWAYS_INLINE void adc(bitblock256_t x, bitblock256_t y, carry_t carry_in, carry_t & carry_out, bitblock256_t & sum)
{
//  Really Slow!
//        bitblock256_t gen = simd_and(x, y);
//        bitblock256_t prop = simd_or(x, y);
//        sum = simd256<256>::add(simd256<256>::add(x, y), carry2bitblock(carry_in));
//        carry_out = bitblock2carry(simd256<256>::srli<255>(simd_or(gen, simd_andc(prop, sum))));

  bitblock128_t x0 = avx_select_lo128(x);
  bitblock128_t x1 = avx_select_hi128(x);
  bitblock128_t y0 = avx_select_lo128(y);
  bitblock128_t y1 = avx_select_hi128(y);
  bitblock128_t c0 = avx_select_lo128(carry2bitblock(carry_in));
  bitblock128_t s0, s1, c1, c2;
  adc128(x0, y0, c0, c1, s0);
  adc128(x1, y1, c1, c2, s1);
  sum = avx_general_combine256(s1, s0);
  carry_out = _mm256_castps128_ps256((__m128) c2);
}
#endif

#ifdef ADCMAGIC

#ifdef AVX2
static inline void adc(bitblock256_t x, bitblock256_t y, carry_t carry_in, carry_t & carry_out, bitblock256_t & sum) {
	bitblock256_t all_ones = simd256<1>::constant<1>();
	bitblock256_t gen = simd_and(x, y);
	bitblock256_t prop = simd_xor(x, y);
  bitblock256_t partial_sum = simd256<64>::add(x, y);
	bitblock256_t carry = simd_or(gen, simd_andc(prop, partial_sum));
  bitblock256_t bubble = simd256<64>::eq(partial_sum, all_ones);
  uint64_t carry_mask = hsimd256<64>::signmask(carry) * 2 + convert(carry_in);
  uint64_t bubble_mask = hsimd256<64>::signmask(bubble);
	uint64_t carry_scan_thru_bubbles = (carry_mask + bubble_mask) &~ bubble_mask;
  uint64_t increments = carry_scan_thru_bubbles | (carry_scan_thru_bubbles - carry_mask);
  carry_out = convert(increments >> 4);
  uint64_t spread = 0x0000200040008001 * increments & 0x0001000100010001;
  sum = simd256<64>::add(partial_sum, _mm256_cvtepu8_epi64(convert_128(spread)));
}
#else
static inline void adc(bitblock256_t x, bitblock256_t y, carry_t carry_in, carry_t & carry_out, bitblock256_t & sum) {
  bitblock128_t all_ones = simd128<1>::constant<1>();
  //bitblock256_t gen = simd_and(x, y);
  //bitblock256_t prop = simd_xor(x, y);
  bitblock128_t x0 = avx_select_lo128(x);
  bitblock128_t x1 = avx_select_hi128(x);
  bitblock128_t y0 = avx_select_lo128(y);
  bitblock128_t y1 = avx_select_hi128(y);
        bitblock128_t sum0 = simd128<64>::add(x0, y0);
        bitblock128_t sum1 = simd128<64>::add(x1, y1);
  //bitblock256_t icarry = simd_or(gen, simd_andc(prop, avx_general_combine256(sum1, sum0)));
        bitblock128_t icarry0 = simd_or(simd_and(x0, y0), simd_andc(simd_or(x0, y0), sum0));
        bitblock128_t icarry1 = simd_or(simd_and(x1, y1), simd_andc(simd_or(x1, y1), sum1));
        // A carry may bubble through a field if it is all ones.
        bitblock128_t bubble0 = simd128<64>::eq(sum0, all_ones);
        bitblock128_t bubble1 = simd128<64>::eq(sum1, all_ones);
        //bitblock128_t bubble = hsimd128<64>::packss(bubble1, bubble0);
        bitblock256_t bubble = avx_general_combine256(bubble1, bubble0);
        //uint64_t carry_mask = _mm256_movemask_pd((__m256d) icarry) * 2 + convert(carry_in);
        uint64_t carry_mask = hsimd128<64>::signmask(icarry1) * 8 + hsimd128<64>::signmask(icarry0) * 2 + convert(carry_in);
        uint64_t bubble_mask = _mm256_movemask_pd((__m256d) bubble);
        //uint64_t bubble_mask = hsimd128<64>::signmask(bubble1) * 4 + hsimd128<64>::signmask(bubble0);
  //uint64_t bubble_mask = hsimd128<32>::signmask(bubble);
  uint64_t carry_scan_thru_bubbles = (carry_mask + bubble_mask) &~ bubble_mask;
        uint64_t increments = carry_scan_thru_bubbles | (carry_scan_thru_bubbles - carry_mask);
        carry_out = convert(increments >> 4);
        uint64_t spread = 0x0000200040008001 * increments & 0x0001000100010001;
        bitblock128_t inc_32 = _mm_cvtepu16_epi32(_mm_cvtsi64_si128(spread));
        bitblock128_t inc_64_0 = esimd128<32>::mergel(simd128<1>::constant<0>(), inc_32);
        bitblock128_t inc_64_1 = esimd128<32>::mergeh(simd128<1>::constant<0>(), inc_32);
        sum = avx_general_combine256(simd128<64>::add(sum1, inc_64_1), simd128<64>::add(sum0, inc_64_0));
}
#endif

#endif

IDISA_ALWAYS_INLINE void sbb128(bitblock128_t x, bitblock128_t y, bitblock128_t borrow_in, bitblock128_t & borrow_out, bitblock128_t & difference)
{
	bitblock128_t gen = simd_andc(y, x);
	bitblock128_t prop = simd_not(simd_xor(x, y));
	bitblock128_t partial = simd128<64>::sub(simd128<64>::sub(x, y), borrow_in);
	bitblock128_t b1 = simd128<128>::slli<64>(simd128<64>::srli<63>(simd_or(gen, simd_and(prop, partial))));
	difference = simd128<64>::sub(partial, b1);
	borrow_out = simd128<128>::srli<127>(simd_or(gen, simd_and(prop, difference)));
}


IDISA_ALWAYS_INLINE void sbb(bitblock256_t x, bitblock256_t y, carry_t borrow_in, carry_t & borrow_out, bitblock256_t & difference)
{
//        bitblock256_t gen = simd_andc(y, x);
//        bitblock256_t prop = simd_not(simd_xor(x, y));
//        difference = simd256<256>::sub(simd256<256>::sub(x, y), carry2bitblock(borrow_in));
//        borrow_out = bitblock2carry(simd256<256>::srli<255>(simd_or(gen, simd_and(prop, difference))));
  bitblock128_t x0 = avx_select_lo128(x);
  bitblock128_t x1 = avx_select_hi128(x);
  bitblock128_t y0 = avx_select_lo128(y);
  bitblock128_t y1 = avx_select_hi128(y);
  bitblock128_t b0 = avx_select_lo128(carry2bitblock(borrow_in));
  bitblock128_t d0, d1, b1, b2;
  sbb128(x0, y0, b0, b1, d0);
  sbb128(x1, y1, b1, b2, d1);
  difference = avx_general_combine256(d1, d0);
  borrow_out = _mm256_castps128_ps256((__m128) b2);
}

IDISA_ALWAYS_INLINE void advance_with_carry128(bitblock128_t cursor, bitblock128_t carry_in, bitblock128_t & carry_out, bitblock128_t & rslt)
{
bitblock128_t shift_out = simd128<64>::srli<63>(cursor);
bitblock128_t low_bits = esimd128<64>::mergel(shift_out, carry_in);
carry_out = simd128<128>::srli<64>(shift_out);
rslt = simd_or(simd128<64>::add(cursor, cursor), low_bits);
}

IDISA_ALWAYS_INLINE void advance_with_carry(bitblock256_t cursor, carry_t carry_in, carry_t & carry_out, bitblock256_t & rslt)
{
  bitblock128_t cursor0 = avx_select_lo128(cursor);
  bitblock128_t cursor1 = avx_select_hi128(cursor);
  bitblock128_t  carry0 = avx_select_lo128(carry_in);
  bitblock128_t  carry1, carry2, rslt0, rslt1;
  advance_with_carry128(cursor0, carry0, carry1, rslt0);
  advance_with_carry128(cursor1, carry1, carry2, rslt1);
  rslt = avx_general_combine256(rslt1, rslt0);
  carry_out = _mm256_castps128_ps256((__m128)carry2);
}
#endif

#endif // BITBLOCK256_HPP_
