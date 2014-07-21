#ifndef TRANSPOSE_HPP
#define TRANSPOSE_HPP

#include "bitblock.hpp"
#include "s2p.hpp"

#ifdef BASIS_BITS
typedef struct Basis_bits {
    BitBlock bit_0;
    BitBlock bit_1;
    BitBlock bit_2;
    BitBlock bit_3;
    BitBlock bit_4;
    BitBlock bit_5;
    BitBlock bit_6;
    BitBlock bit_7;
} Basis_bits;
#endif

static inline void s2p_do_block(BytePack U8[], Basis_bits & basis_bits);
static inline void s2p_do_final_block(BytePack U8[], Basis_bits & basis_bits, BitBlock EOF_mask);

static IDISA_ALWAYS_INLINE void s2p_do_block(BytePack U8[], Basis_bits & basis_bits) {
  s2p(U8[0], U8[1], U8[2], U8[3], U8[4], U8[5], U8[6], U8[7],
    basis_bits.bit_0, basis_bits.bit_1, basis_bits.bit_2, basis_bits.bit_3, basis_bits.bit_4, basis_bits.bit_5, basis_bits.bit_6, basis_bits.bit_7);
}

static IDISA_ALWAYS_INLINE void s2p_do_final_block(BytePack U8[], Basis_bits & basis_bits, BitBlock EOF_mask) {
  s2p_do_block(U8, basis_bits);
  basis_bits.bit_0 = simd_and(basis_bits.bit_0, EOF_mask);
  basis_bits.bit_1 = simd_and(basis_bits.bit_1, EOF_mask);
  basis_bits.bit_2 = simd_and(basis_bits.bit_2, EOF_mask);
  basis_bits.bit_3 = simd_and(basis_bits.bit_3, EOF_mask);
  basis_bits.bit_4 = simd_and(basis_bits.bit_4, EOF_mask);
  basis_bits.bit_5 = simd_and(basis_bits.bit_5, EOF_mask);
  basis_bits.bit_6 = simd_and(basis_bits.bit_6, EOF_mask);
  basis_bits.bit_7 = simd_and(basis_bits.bit_7, EOF_mask);
}

struct Transpose {
  Transpose() {}
  void do_block(char * bytedata, Basis_bits & basis_bits) {
    s2p_do_block((BytePack *) bytedata, basis_bits);
  }
  void do_final_block(char * bytedata, Basis_bits & basis_bits, BitBlock EOF_mask) {
    s2p_do_final_block((BytePack *) bytedata, basis_bits, EOF_mask);
  }
  void clear() {}
};
#endif // TRANSPOSE_HPP

