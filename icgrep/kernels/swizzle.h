/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SWIZZLE_H
#define SWIZZLE_H

#include "kernel.h"
namespace IDISA { class IDISA_Builder; }


// The SwizzleGenerator class creates a kernel that transforms a set of bit streams into a swizzled form.
// In swizzled form, one "swizzle field" each from a set of streams are grouped together to be processed 
// as a unit using SIMD operations.   For example, for a swizzle field width of 64 and a block size of 256,
// 4 streams are swizzled together to be operated on as a group.  The ratio of the block size to the
// swizzle field size is known as the swizzle factor, in this case 4.

// Swizzled form is convenient for performing sequential operations on parallel sets of streams,
// such as compression of each swizzle field by known counts, or stitching together known numbers
// of bits from two different sources.

// Any number of bit streams may be swizzled together.  However, the outputSets are always
// grouped together in multiples of the swizzle factor.   If the bit stream count is not
// an exact multiple of the swizzle factor, null streams (all zero bits) are added for
// each swizzle group.

// The input streams may come from any number of parallel input sets, each of the same size.
// The number of inputSets defaults to 1.

// The output streams may be separated in a number of output sets.  However, output streams
// sets must be a multiple of the swizzle factor size.

// For example: consider the following 4 streams (32 bits each)
// Stream 1:   abcdef00 ghi00000 jk000000 lmnop000
// Stream 2:   qrstuv00 wxy00000 z1000000 23456000
// Stream 3:   ABCDEF00 GHI00000 JK000000 LMNOP000
// Stream 4:   QRSTUV00 WZY00000 Z1000000 23456000
//
// The swizzled output using a field width of 8 produces the following swizzles.
//
// Swizzle 1:  abcdef00 qrstuv00 ABCDEF00 QRSTUV00
// Swizzle 2:  ghi00000 wxy00000 GHI00000 WZY00000
// Swizzle 3:  jk000000 z1000000 JK000000 Z1000000
// Swizzle 4:  lmnop000 23456000 LMNOP000 23456000
//
// Now it might be convenient to compress all fields of swizzle 1 by 2, all fields of swizzle 2 by 5
// and so on.
//
namespace kernel {

class SwizzleGenerator : public BlockOrientedKernel {
public:
    
    SwizzleGenerator(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned bitStreamCount, unsigned outputSets = 1, unsigned inputSets = 1, unsigned fieldWidth = 64);
    
protected:
    
    void generateDoBlockMethod() override;
    
private:
    const unsigned mBitStreamCount;
    const unsigned mFieldWidth;
    const unsigned mSwizzleFactor;
    const unsigned mInputSets;
    const unsigned mOutputSets;
};

}
    
#endif

