#ifndef CARRYQ_HPP_
#define CARRYQ_HPP_

/*=============================================================================
  carryQ.hpp - Pablo compiler support for carry introduction.
  Ken Herdy, Robert D. Cameron
  Copyright (C) 2012, Robert D. Cameron, Kenneth S. Herdy.
    Licensed to the public under the Open Software License 3.0.
    Licensed to International Characters Inc.
       under the Academic Free License version 3.0.
  April 2012
=============================================================================*/

#include <string.h>
#include <stdint.h>
#include <iostream>

#include "bitblock.hpp"
#include "stdio.h"

///////////////////////////////////////////////////////////////////////////////
//
// Carry method variants.
//
// BitBlock_op_ci_co() 	- standard block non while loop statement and in final block if ignore the carry out
// BitBlock_op_co() 	- standard block while loop and in final block while loop if ignore carry out
// BitBlock_op_ci()		- final block non while loop statement
// BitBlock_op()		- final while loop statement
//
// BitBlock_op_ci(), BitBlock_op() methods not implemented to reduce the total number of
// methods and Pablo compiler complexity.
//
///////////////////////////////////////////////////////////////////////////////

#define interpose32(x,y,pos) interpose32_<pos>(x,y)	
template<uint32_t n>
IDISA_ALWAYS_INLINE BitBlock interpose32_(BitBlock s, BitBlock s32) {
	return simd_or(simd<32>::slli<n>(s), simd<32>::srli<32-n>(s32));
}

template<uint32_t n>
IDISA_ALWAYS_INLINE BitBlock interpose64_(BitBlock s, BitBlock s64) {
	return simd_or(simd<64>::slli<n>(s), simd<64>::srli<64-n>(s64));
}

template <uint16_t CarryCount, uint16_t AdvanceNCount> class CarryArray;

#define LocalCarryCombine(carrySet, localCarry, carryNo, carryCount)\
	carrySet.CarryCombine(localCarry.cq, carryNo, carryCount);  

#define CarryDeclare(name, carry1_count, carryN_count)\
CarryArray<carry1_count, carryN_count> name;

#define LocalCarryDeclare(name, count)\
CarryArray<count, 0> name;

// Array of BitBlock implementation.
template <uint16_t CarryCount, uint16_t AdvanceNCount>
class CarryArray {

public:

#ifndef CARRY64
    #define carry_value_0 simd<BLOCK_SIZE>::constant<0>()
    #define carry_value_1 simd<BLOCK_SIZE>::constant<1>()
#endif
#ifdef CARRY64
    #define carry_value_0 0
    #define carry_value_1 1
#endif

	carry_t cq[CarryCount + AdvanceNCount];
	//BitBlock pending64[AdvanceNCount];
	CarryArray()
	{
	    memset (cq, 0, sizeof(carry_t) * (CarryCount + AdvanceNCount));
	    //memset(pending64, 0, sizeof(BitBlock) * AdvanceNCount);
	}
	~CarryArray() {}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_advance_ci_co(BitBlock strm, carry_t carryin, uint16_t carryno)
	{
		BitBlock rslt;
		advance_with_carry(strm, carryin, cq[carryno], rslt);
		return rslt;
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_add_ci_co(BitBlock strm1, BitBlock strm2, carry_t carryin, const uint16_t carryno)
	{
		BitBlock sum;
		adc(strm1, strm2, carryin, cq[carryno], sum);
		return sum;
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_sub_ci_co(BitBlock strm1, BitBlock strm2, carry_t carryin, uint16_t carryno)
	{
		BitBlock diff;
		sbb(strm1, strm2, carryin, cq[carryno], diff);
		return diff;
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_scantofirst(BitBlock charclass, carry_t carryin, uint16_t carryno)
	{
		BitBlock marker;
//		BitBlock c = carry_flip(carryin);
	    	adc(simd<BLOCK_SIZE>::constant<0>(), simd_not(charclass), carryin, cq[carryno], marker);
//	    	cq[carryno] = carry_flip(cq[carryno]);
	    	return simd_and(marker, charclass);
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_scanthru_ci_co(BitBlock markers0, BitBlock charclass, carry_t carryin, uint16_t carryno)
	{
		BitBlock markers1;
		adc(markers0, charclass, carryin, cq[carryno], markers1);
		return simd_andc(markers1, charclass);
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_advance_then_scanthru(BitBlock markers0, BitBlock charclass, carry_t carryin, uint16_t carryno)
	{
		BitBlock markers1;
		//assert(!bitblock::any(simd_and(markers0, charclass)));
		adc(markers0, simd_or(charclass, markers0), carryin, cq[carryno], markers1);
		return simd_andc(markers1, charclass);
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_span_upto(BitBlock starts, BitBlock follows, carry_t carryin, uint16_t carryno)
	{
		BitBlock span;
		sbb(follows, starts, carryin, cq[carryno], span);
		return span;
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_inclusive_span(BitBlock starts, BitBlock ends, carry_t carryin, uint16_t carryno)
	{
		BitBlock span;
		sbb(ends, starts, carryin, cq[carryno], span);
		return simd_or(span, ends);
	}

	IDISA_ALWAYS_INLINE BitBlock BitBlock_exclusive_span(BitBlock starts, BitBlock ends, carry_t carryin, uint16_t carryno)
	{
		BitBlock span;
		sbb(ends, starts, carryin, cq[carryno], span);
		return simd_andc(span, starts);
	}

/*
	IDISA_ALWAYS_INLINE BitBlock BitBlock_advance32_ci_co(BitBlock strm, uint32_t pending_in, uint32_t & pending_out)
	{
		pending_out = (uint32_t) mvmd<32>::extract< (sizeof(BitBlock)/sizeof(pending_out))-1 >(strm);
		return simd_or(simd<BLOCK_SIZE>::slli<32>(strm), mvmd<BLOCK_SIZE>::fill((uint64_t)pending_in));
	}
*/

/*
	template <int n> IDISA_ALWAYS_INLINE BitBlock BitBlock_advance_n_(BitBlock strm, carry_t pending_in, uint16_t pendingno)
        {
		BitBlock half_block_shifted = esimd<BLOCK_SIZE/2>::mergel(strm, pending_in);
		cq[CarryCount + pendingno] = bitblock::srli<BLOCK_SIZE/2>(strm);
		//pending64[pendingno] = bitblock::srli<BLOCK_SIZE/2>(strm);
		BitBlock result = simd_or(simd<BLOCK_SIZE/2>::srli<(BLOCK_SIZE/2)-n>(half_block_shifted),
			       simd<BLOCK_SIZE/2>::slli<n>(strm));
		return result;
        }
*/

	IDISA_ALWAYS_INLINE bool CarryTest(uint16_t carryno, uint16_t carry_count)
	{
		  carry_t c1 = cq[carryno];
		  int ubound = carryno + carry_count;
		  for (int i = carryno + 1; i < ubound ; i++) {
			c1 = carry_or(c1, cq[i]);
		  }
		  return test_carry(c1);
	}

	IDISA_ALWAYS_INLINE carry_t CarryRange(uint16_t carryno, uint16_t carry_count)
	{
		  carry_t c1 = cq[carryno];
		  int ubound = carryno + carry_count;
		  for (int i = carryno + 1; i < ubound ; i++) {
			c1 = carry_or(c1, cq[i]);
		  }
		  return c1;
	}

	IDISA_ALWAYS_INLINE void CarryDequeueEnqueue(uint16_t carryno, uint16_t carry_count)
	{
		return;
	}

	// Deprecated (renamed)
	IDISA_ALWAYS_INLINE void CarryQ_Adjust(uint16_t carry_count)
	{
		return;
	}

	IDISA_ALWAYS_INLINE void CarryAdjust(uint16_t carry_count)
	{
		return;
	}

	IDISA_ALWAYS_INLINE void CarryCombine(carry_t local_cq[], uint16_t carryno, uint16_t carry_count)
	{
		  for (int i = 0; i < carry_count; i++) {
		    cq[carryno+i] = carry_or(cq[carryno+i], local_cq[i]);
		  }
	}

	IDISA_ALWAYS_INLINE void CarryCombine1(uint16_t carryno, uint16_t carry2)
	{
		  cq[carryno] = carry_or(cq[carryno], cq[carry2]);
		  cq[carry2] = carry_value_0;
	}

	IDISA_ALWAYS_INLINE carry_t get_carry_in(uint16_t carryno) const 
	{
		return cq[carryno];
	}

	// Deprecated (renamed)
	IDISA_ALWAYS_INLINE carry_t GetCarry(uint16_t carryno) const
	{
		return cq[carryno];
	}

	IDISA_ALWAYS_INLINE void SetCarry(carry_t carryVal, uint16_t carryno)
	{
		cq[carryno] = carryVal;
	}


        // Deprecated in PabloJ, retained for legacy compiler.
	IDISA_ALWAYS_INLINE carry_t get_pending64(uint16_t advance_n_blkno) const 
	{
		return cq[CarryCount + advance_n_blkno];
	}

	IDISA_ALWAYS_INLINE carry_t Pending64(uint16_t advance_n_blkno) const 
	{
		return cq[CarryCount + advance_n_blkno];
	}

//private:
	// helpers
#ifndef  CARRY64
	// Deprecated (renamed)
	IDISA_ALWAYS_INLINE carry_t carry_flip(carry_t carry) const
	{
		return simd_xor(carry, carry_value_1);
	}

	IDISA_ALWAYS_INLINE BitBlock CarryFlip(carry_t carry) const
	{
		return simd_xor(carry, carry_value_1);
	}

	IDISA_ALWAYS_INLINE bool test_carry(carry_t carry) const
	{
		return bitblock::any(carry);
	}

	IDISA_ALWAYS_INLINE carry_t carry_or(carry_t carry1, carry_t carry2) const
	{
		return simd_or(carry1, carry2);
	}
#endif
#ifdef  CARRY64
	// Deprecated (renamed)
	IDISA_ALWAYS_INLINE carry_t carry_flip(carry_t carry) const
	{
		return carry ^ 1;
	}

	IDISA_ALWAYS_INLINE BitBlock CarryFlip(carry_t carry) const
	{
		return carry ^ 1;
	}

	IDISA_ALWAYS_INLINE bool test_carry(carry_t carry) const
	{
		return carry != 0;
	}

	IDISA_ALWAYS_INLINE carry_t carry_or(carry_t carry1, carry_t carry2) const
	{
		return carry1 | carry2;
	}
#endif

};

#endif // CARRYQ_HPP_
