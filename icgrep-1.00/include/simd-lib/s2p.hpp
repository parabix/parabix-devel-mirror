/*  s2p - Serial to Parallel Bit Stream Transposition
    Copyright (c) 2007, 2008, 2010, 2011  Robert D. Cameron.
    Licensed to the public under the Open Software License 3.0.
    Licensed to International Characters Inc.
       under the Academic Free License version 3.0.
*/

#ifndef S2P_HPP
#define S2P_HPP

#include "idisa.hpp"

#define BytePack BitBlock

/* Given a block of bytes in 8 consecutive registers s0, s1, ..., s7,
   s2p transposes the block into 8 parallel bitstream blocks p0, p1, ..., p7.

   The following header shows the intent, although a macro is used for
   speed.
static inline void s2p(BytePack s0, BytePack s1, BytePack s2, BytePack s3,
                       BytePack s5, BytePack s6, BytePack s7, BytePack s8,
                       BitBlock& p0, BitBlock& p1, BitBlock& p2, BitBlock& p3, 
                       BitBlock& p4, BitBlock& p5, BitBlock& p6, BitBlock& p7);
*/

/*  1.  ALGORITHM Selection.  
        Choice of 3 algorithms: s2p_ideal, s2p_movemask, s2p_bytepack
        Default is s2p_bytepack.
        Compiling with -DUSE_S2P_IDEAL or -DUSE_S2P_MOVEMASK to override.
*/

#ifdef USE_S2P_IDEAL
#define S2P_ALGORITHM s2p_ideal
#endif

#ifdef USE_S2P_MOVEMASK
#define S2P_ALGORITHM s2p_movemask
#endif

#ifndef S2P_ALGORITHM
#define S2P_ALGORITHM s2p_bytepack
#endif

#define s2p(s0, s1, s2, s3, s4, s5, s6, s7, p0, p1, p2, p3, p4, p5, p6, p7)\
  S2P_ALGORITHM(s7, s6, s5, s4, s3, s2, s1, s0, p0, p1, p2, p3, p4, p5, p6, p7)

/*  s2p_ideal is an ideal serial to parallel transposition
    algorithm given an architecture with native support for
    simd_pack_{8,4,2}_{hh,ll} operations, achieving transposition
    of 8 serial bytepacks into 8 parallel bitblocks in only 24 pack
    operations.
*/

#define s2p_ideal(s0, s1, s2, s3, s4, s5, s6, s7, p0, p1, p2, p3, p4, p5, p6, p7) \
  do {\
	BitBlock bit0123_0, bit0123_1, bit0123_2, bit0123_3,\
	bit4567_0, bit4567_1, bit4567_2, bit4567_3;\
	BitBlock bit01_0, bit01_1, bit23_0, bit23_1, bit45_0, bit45_1, bit67_0, bit67_1;\
	bit0123_0 = hsimd<8>::packh(s0, s1);\
	bit0123_1 = hsimd<8>::packh(s2, s3);\
	bit0123_2 = hsimd<8>::packh(s4, s5);\
	bit0123_3 = hsimd<8>::packh(s6, s7);\
	bit4567_0 = hsimd<8>::packl(s0, s1);\
	bit4567_1 = hsimd<8>::packl(s2, s3);\
	bit4567_2 = hsimd<8>::packl(s4, s5);\
	bit4567_3 = hsimd<8>::packl(s6, s7);\
	bit01_0 = hsimd<4>::packh(bit0123_0, bit0123_1);\
	bit01_1 = hsimd<4>::packh(bit0123_2, bit0123_3);\
	bit23_0 = hsimd<4>::packl(bit0123_0, bit0123_1);\
	bit23_1 = hsimd<4>::packl(bit0123_2, bit0123_3);\
	bit45_0 = hsimd<4>::packh(bit4567_0, bit4567_1);\
	bit45_1 = hsimd<4>::packh(bit4567_2, bit4567_3);\
	bit67_0 = hsimd<4>::packl(bit4567_0, bit4567_1);\
	bit67_1 = hsimd<4>::packl(bit4567_2, bit4567_3);\
	p0 = hsimd<2>::packh(bit01_0, bit01_1);\
	p1 = hsimd<2>::packl(bit01_0, bit01_1);\
	p2 = hsimd<2>::packh(bit23_0, bit23_1);\
	p3 = hsimd<2>::packl(bit23_0, bit23_1);\
	p4 = hsimd<2>::packh(bit45_0, bit45_1);\
	p5 = hsimd<2>::packl(bit45_0, bit45_1);\
	p6 = hsimd<2>::packh(bit67_0, bit67_1);\
	p7 = hsimd<2>::packl(bit67_0, bit67_1);\
  } while(0)


/*  s2p_bytepack is a fast serial to parallel transposition
    algorithm given an architecture with simd_pack_16 operations,
    but not at small field widths.
    MMX, SSE, Altivec ...
*/


#ifndef USE_S2P_AVX
#define s2p_step(s0, s1, hi_mask, shift, p0, p1)  \
  do {\
	BitBlock t0,t1;\
	t0 = hsimd<16>::packh(s0, s1);\
	t1 = hsimd<16>::packl(s0, s1);\
	p0 = simd<1>::ifh(hi_mask, t0, simd<16>::srli<shift>(t1));\
	p1 = simd<1>::ifh(hi_mask, simd<16>::slli<shift>(t0), t1);\
  } while(0)
#endif


/* For AVX, we use a modified s2p_step function to avoid a number
   of conversions from 128-bit mode to 256-bit mode just to 
   immediately convert back. */
#ifdef USE_S2P_AVX
#define s2p_step(s0, s1, hi_mask, shift, p0, p1)  \
  do {\
	bitblock128_t s00, s01, s10, s11, t00, t01, t10, t11;\
	bitblock128_t t10shift, t11shift, t00shift, t01shift;\
	s00 = avx_select_hi128(s0);\
	s01 = avx_select_lo128(s0);\
	s10 = avx_select_hi128(s1);\
	s11 = avx_select_lo128(s1);\
	t00 = hsimd128<16>::packh(s00, s01);\
	t10 = hsimd128<16>::packl(s00, s01);\
	t01 = hsimd128<16>::packh(s10, s11);\
	t11 = hsimd128<16>::packl(s10, s11);\
	t10shift = simd128<16>::srli<shift>(t10);\
	t11shift = simd128<16>::srli<shift>(t11);\
	t00shift = simd128<16>::slli<shift>(t00);\
	t01shift = simd128<16>::slli<shift>(t01);\
	p0 = simd<1>::ifh(hi_mask, avx_general_combine256(t00, t01), avx_general_combine256(t10shift, t11shift));\
	p1 = simd<1>::ifh(hi_mask, avx_general_combine256(t00shift, t01shift), avx_general_combine256(t10, t11));\
  } while(0)
#endif

#ifndef USE_S2P_AVX2
#define s2p_bytepack(s0, s1, s2, s3, s4, s5, s6, s7, p0, p1, p2, p3, p4, p5, p6, p7) \
  do {\
	BitBlock bit00224466_0,bit00224466_1,bit00224466_2,bit00224466_3;\
	BitBlock bit11335577_0,bit11335577_1,bit11335577_2,bit11335577_3;\
	BitBlock bit00004444_0,bit22226666_0,bit00004444_1,bit22226666_1;\
	BitBlock bit11115555_0,bit33337777_0,bit11115555_1,bit33337777_1;\
	s2p_step(s0,s1,simd<2>::himask(),1,bit00224466_0,bit11335577_0);\
	s2p_step(s2,s3,simd<2>::himask(),1,bit00224466_1,bit11335577_1);\
	s2p_step(s4,s5,simd<2>::himask(),1,bit00224466_2,bit11335577_2);\
	s2p_step(s6,s7,simd<2>::himask(),1,bit00224466_3,bit11335577_3);\
	s2p_step(bit00224466_0,bit00224466_1,simd<4>::himask(),2,bit00004444_0,bit22226666_0);\
	s2p_step(bit00224466_2,bit00224466_3,simd<4>::himask(),2,bit00004444_1,bit22226666_1);\
	s2p_step(bit11335577_0,bit11335577_1,simd<4>::himask(),2,bit11115555_0,bit33337777_0);\
	s2p_step(bit11335577_2,bit11335577_3,simd<4>::himask(),2,bit11115555_1,bit33337777_1);\
	s2p_step(bit00004444_0,bit00004444_1,simd<8>::himask(),4,p0,p4);\
	s2p_step(bit11115555_0,bit11115555_1,simd<8>::himask(),4,p1,p5);\
	s2p_step(bit22226666_0,bit22226666_1,simd<8>::himask(),4,p2,p6);\
	s2p_step(bit33337777_0,bit33337777_1,simd<8>::himask(),4,p3,p7);\
  } while(0)
#endif

#ifdef USE_S2P_AVX2
#define s2p_step_shuf(shuf, s0, s1, hi_mask, shift, p0, p1)  \
  do {\
	BitBlock x0, x1, t0, t1;\
        x0 = _mm256_permute4x64_epi64(_mm256_shuffle_epi8(s0, shuf), 0xD8);\
        x1 = _mm256_permute4x64_epi64(_mm256_shuffle_epi8(s1, shuf), 0xD8);\
        t0 = _mm256_permute2x128_si256(x1, x0, 0x31);\
        t1 = _mm256_permute2x128_si256(x1, x0, 0x20);\
        p0 = simd<1>::ifh(hi_mask, t0, simd<16>::srli<shift>(t1));\
        p1 = simd<1>::ifh(hi_mask, simd<16>::slli<shift>(t0), t1);\
  } while(0)

#define s2p_bytepack(s0, s1, s2, s3, s4, s5, s6, s7, p0, p1, p2, p3, p4, p5, p6, p7) \
  do {\
        BitBlock shuf = _mm256_set_epi32(0x0F0D0B09, 0x07050301, 0x0E0C0A08, 0x06040200, 0x0F0D0B09, 0x07050301, 0x0E0C0A08, 0x06040200);\
	BitBlock bit00224466_0,bit00224466_1,bit00224466_2,bit00224466_3;\
	BitBlock bit11335577_0,bit11335577_1,bit11335577_2,bit11335577_3;\
	BitBlock bit00004444_0,bit22226666_0,bit00004444_1,bit22226666_1;\
	BitBlock bit11115555_0,bit33337777_0,bit11115555_1,bit33337777_1;\
	s2p_step_shuf(shuf, s0,s1,simd<2>::himask(),1,bit00224466_0,bit11335577_0);\
	s2p_step_shuf(shuf, s2,s3,simd<2>::himask(),1,bit00224466_1,bit11335577_1);\
	s2p_step_shuf(shuf, s4,s5,simd<2>::himask(),1,bit00224466_2,bit11335577_2);\
	s2p_step_shuf(shuf, s6,s7,simd<2>::himask(),1,bit00224466_3,bit11335577_3);\
	s2p_step_shuf(shuf, bit00224466_0,bit00224466_1,simd<4>::himask(),2,bit00004444_0,bit22226666_0);\
	s2p_step_shuf(shuf, bit00224466_2,bit00224466_3,simd<4>::himask(),2,bit00004444_1,bit22226666_1);\
	s2p_step_shuf(shuf, bit11335577_0,bit11335577_1,simd<4>::himask(),2,bit11115555_0,bit33337777_0);\
	s2p_step_shuf(shuf, bit11335577_2,bit11335577_3,simd<4>::himask(),2,bit11115555_1,bit33337777_1);\
	s2p_step_shuf(shuf, bit00004444_0,bit00004444_1,simd<8>::himask(),4,p0,p4);\
	s2p_step_shuf(shuf, bit11115555_0,bit11115555_1,simd<8>::himask(),4,p1,p5);\
	s2p_step_shuf(shuf, bit22226666_0,bit22226666_1,simd<8>::himask(),4,p2,p6);\
	s2p_step_shuf(shuf, bit33337777_0,bit33337777_1,simd<8>::himask(),4,p3,p7);\
  } while(0)

#endif




/* For sizeof(BitBlock) = 16 */
#if BLOCK_SIZE == 128
typedef uint16_t BitPack;
#endif
#if BLOCK_SIZE == 256
typedef uint32_t BitPack;
#endif

#define movemask_step(s7, s6, s5, s4, s3, s2, s1, s0, p) \
  do { \
	union { BitPack bit_pack[8];\
       		BitBlock bit_block;\
	      } b;\
	b.bit_pack[0] = hsimd<8>::signmask(s0);\
	b.bit_pack[1] = hsimd<8>::signmask(s1);\
	b.bit_pack[2] = hsimd<8>::signmask(s2);\
	b.bit_pack[3] = hsimd<8>::signmask(s3);\
	b.bit_pack[4] = hsimd<8>::signmask(s4);\
	b.bit_pack[5] = hsimd<8>::signmask(s5);\
	b.bit_pack[6] = hsimd<8>::signmask(s6);\
	b.bit_pack[7] = hsimd<8>::signmask(s7);\
	p = b.bit_block;\
   } while (0)

#define bitshift_step(s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7) \
  do { \
	t0 = simd<8>::add(s0, s0);\
	t1 = simd<8>::add(s1, s1);\
	t2 = simd<8>::add(s2, s2);\
	t3 = simd<8>::add(s3, s3);\
	t4 = simd<8>::add(s4, s4);\
	t5 = simd<8>::add(s5, s5);\
	t6 = simd<8>::add(s6, s6);\
	t7 = simd<8>::add(s7, s7);\
  } while (0)


#define s2p_movemask(s0, s1, s2, s3, s4, s5, s6, s7, p0, p1, p2, p3, p4, p5, p6, p7) \
  do { \
	BitBlock t0, t1, t2, t3, t4, t5, t6, t7;\
	movemask_step(s0, s1, s2, s3, s4, s5, s6, s7, p0);\
	bitshift_step(s0, s1, s2, s3, s4, s5, s6, s7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p1);\
	bitshift_step(t0, t1, t2, t3, t4, t5, t6, t7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p2);\
	bitshift_step(t0, t1, t2, t3, t4, t5, t6, t7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p3);\
	bitshift_step(t0, t1, t2, t3, t4, t5, t6, t7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p4);\
	bitshift_step(t0, t1, t2, t3, t4, t5, t6, t7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p5);\
	bitshift_step(t0, t1, t2, t3, t4, t5, t6, t7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p6);\
	bitshift_step(t0, t1, t2, t3, t4, t5, t6, t7, t0, t1, t2, t3, t4, t5, t6, t7);\
	movemask_step(t0, t1, t2, t3, t4, t5, t6, t7, p7);\
  } while (0)


#endif // S2P_HPP

