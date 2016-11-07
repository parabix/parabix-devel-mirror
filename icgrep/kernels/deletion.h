/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELETION_H
#define DELETION_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"


namespace llvm { class Module; class Value;}

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
// Deletion Mask Calculation

std::vector<llvm::Value *> parallel_prefix_deletion_masks(IDISA::IDISA_Builder * iBuilder, unsigned fw, llvm::Value * del_mask);

// Applying Deletion Masks to a Stream

llvm::Value * apply_parallel_prefix_deletion(IDISA::IDISA_Builder * iBuilder, unsigned fw, llvm::Value * del_mask, std::vector<llvm::Value *> mv, llvm::Value * strm);

using namespace parabix;

namespace kernel {

class DeletionKernel : public kernel::KernelBuilder {
public:
    DeletionKernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount) :
    KernelBuilder(iBuilder, "del",
                  {Binding{StreamSetType(iBuilder,streamCount + 2, 1), "inputStreamSet"}},
                  {Binding{StreamSetType(iBuilder,streamCount, 1), "outputStreamSet"},
                   Binding{StreamSetType(iBuilder,1, 1), "deletionCounts"}},
                  {}, {}, {}),
    mDeletionFieldWidth(fw),
    mStreamCount(streamCount) {}
    
private:
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod() override;
    unsigned mDeletionFieldWidth;
    unsigned mStreamCount;
};

}
    
#endif

