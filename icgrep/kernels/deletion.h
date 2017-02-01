/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELETION_H
#define DELETION_H

#include "kernel.h"
namespace IDISA { class IDISA_Builder; }

//
// Parallel Prefix Deletion 
// see Parallel Prefix Compress in Henry S. Warren, Hacker's Delight, Chapter 7
// 
// Given that we want to delete bits within fields of width fw, moving
// nondeleted bits to the right, the parallel prefix compress method can
// be applied.   This requires a preprocessing step to compute log2(fw) 
// masks that can be used to select bits to be moved in each step of the
// algorithm.
//

namespace kernel {

class DeletionKernel : public BlockOrientedKernel {
public:

    DeletionKernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount);
    
protected:

    void generateDoBlockMethod(llvm::Value * blockNo) override;

    void generateFinalBlockMethod(llvm::Value * remainingBytes, llvm::Value * blockNo) override;

private:
    const unsigned mDeletionFieldWidth;
    const unsigned mStreamCount;
};

}
    
#endif

