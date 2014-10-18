/*=============================================================================
  pabloSupport.hpp - Pablo compiler support for carry introduction.
  Will replace deprecated carryQ.hpp
  Copyright (C) 2012, Robert D. Cameron
    Licensed to the public under the Open Software License 3.0.
    Licensed to International Characters Inc.
       under the Academic Free License version 3.0.
  December 2012
=============================================================================*/
#ifndef PABLOSUPPORT_HPP_
#define PABLOSUPPORT_HPP_

#include "bitblock.hpp"

#define BitBlock_declare(name)  BitBlock name

#define ubitblock_declare(name, n) \
  ubitblock name[n];\
  do {int i;\
      for (i = 0; i < n; i++) name[i]._128 = simd<1>::constant<0>();\
     }\
  while (0)
     

const BitBlock simd_const_1 = mvmd<BLOCK_SIZE/4>::fill4(0,0,0,1); 

const BitBlock simd_sign_bit = bitblock::slli<BLOCK_SIZE-1>(simd_const_1); 

IDISA_ALWAYS_INLINE BitBlock Dequeue_bit(BitBlock & q) {
        BitBlock bit = simd_and(q, simd_const_1);
        q = simd<64>::srli<1>(q);
        return bit;
} 

IDISA_ALWAYS_INLINE carry_t pablo_blk_Advance(BitBlock strm, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;
	adv_ci_co(strm, carryin, carryout, rslt);
	return carryout;
}

template <int n> IDISA_ALWAYS_INLINE carry_t pablo_blk_Advance_n_(BitBlock strm, BitBlock pending_in, BitBlock & rslt) {
	BitBlock half_block_shifted = esimd<BLOCK_SIZE/2>::mergel(strm, pending_in);
	rslt = simd_or(simd<BLOCK_SIZE/2>::srli<(BLOCK_SIZE/2)-n>(half_block_shifted),
		       simd<BLOCK_SIZE/2>::slli<n>(strm));
	return strm;
}

/*  Support for pablo.Lookahead.  2 translation modes:
    (a) pablo.Lookahead(ss.strm, n) ==> pablo_blk_Lookahead_n_<n>(ss_curblock.strm, ss_nxtblock.strm);
    (b) pablo.Lookahead(ss.strm, n) ==> pablo_blk_Lookahead_n_<n>(ss[0].strm, ss[1].strm);
*/
template <int n> IDISA_ALWAYS_INLINE carry_t pablo_blk_Lookahead_n_(BitBlock strm, BitBlock lookahead) {
	BitBlock half_block_shifted = mvmd<BLOCK_SIZE/2>::dslli<1>(lookahead, strm);
	return simd_or(simd<BLOCK_SIZE/2>::slli<(BLOCK_SIZE/2)-n>(half_block_shifted),
		       simd<BLOCK_SIZE/2>::srli<n>(strm));
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_ScanThru(BitBlock marker, BitBlock charclass, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;  BitBlock sum;
	add_ci_co(marker, charclass, carryin, carryout, sum);
        rslt = simd_andc(sum, charclass);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_MatchStar(BitBlock marker, BitBlock charclass, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;  BitBlock sum;
	add_ci_co(simd_and(charclass, marker), charclass, carryin, carryout, sum);
        rslt = simd_or(simd_xor(sum, charclass), marker);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_AdvanceThenScanThru(BitBlock marker, BitBlock charclass, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;  BitBlock sum;
	add_ci_co(marker, simd_or(charclass, marker), carryin, carryout, sum);
        rslt = simd_andc(sum, charclass);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_ScanTo(BitBlock marker, BitBlock charclass, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;  BitBlock sum;
	add_ci_co(marker, simd_not(charclass), carryin, carryout, sum);
        rslt = simd_and(sum, charclass);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_AdvanceThenScanTo(BitBlock marker, BitBlock charclass, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;  BitBlock sum;
	add_ci_co(marker, simd_or(marker, simd_not(charclass)), carryin, carryout, sum);
        rslt = simd_and(sum, charclass);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_ScanToFirst(BitBlock charclass, carry_t carryin, BitBlock & rslt) {
        carry_t carryout;  BitBlock sum;
	add_ci_co(simd<BLOCK_SIZE>::constant<0>(), simd_not(charclass), carryin, carryout, sum);
        rslt = simd_and(sum, charclass);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_SpanUpTo(BitBlock starts, BitBlock follows, carry_t carryin, BitBlock & rslt) {
	carry_t carryout;
	sub_bi_bo(follows, starts, carryin, carryout, rslt);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_InclusiveSpan(BitBlock starts, BitBlock follows, carry_t carryin, BitBlock & rslt) {
	carry_t carryout;  BitBlock span;
	sub_bi_bo(follows, starts, carryin, carryout, span);
	rslt = simd_or(span, follows);
	return carryout;
}

IDISA_ALWAYS_INLINE carry_t pablo_blk_ExclusiveSpan(BitBlock starts, BitBlock follows, carry_t carryin, BitBlock & rslt) {
	carry_t carryout;  BitBlock span;
	sub_bi_bo(follows, starts, carryin, carryout, span);
	rslt = simd_andc(span, starts);
	return carryout;
}

// KH: expects bitblock::any(v)
IDISA_ALWAYS_INLINE uint64_t pablo_blk_IndexOf(BitBlock v, int block_base) {
    union {BitBlock bitblock; ScanWord elems[sizeof(BitBlock)/sizeof(ScanWord)];} u;
	u.bitblock = v;    
    int pos;
    ScanWord s;
    for (unsigned int i = 0; i < sizeof(BitBlock)/sizeof(ScanWord); i++) {
        s = u.elems[i];
        if (s != 0) {
            int bitpos = scan_forward_zeroes(s);
            pos = block_base + (i * sizeof(ScanWord) * 8) + bitpos;
        }
    }
    
	return pos;
}

template <typename T> IDISA_ALWAYS_INLINE BitBlock pablo_blk_match(T bytedata[], const T match_str[], BitBlock v, int len) {
	union {BitBlock bitblock; ScanWord elems[sizeof(BitBlock)/sizeof(ScanWord)];} u;
	u.bitblock = v;
	int pos;
	ScanWord s, t, bit;
	for (unsigned int i = 0; i < sizeof(BitBlock)/sizeof(ScanWord); i++) {
		s = u.elems[i];
		while (s != 0) {
			pos = scan_forward_zeroes(s);
			t = s;
			s = s & (s - 1); // clear rightmost bit
			if (memcmp((void *) &bytedata[pos], (void *) match_str, len * sizeof(T))) {
                                // Strings do not match; filter the result.
				bit = s ^ t;
				u.elems[i] ^= bit;
			}
		}
	}
	return u.bitblock;
}

#endif // PABLOSUPPORT_HPP_
