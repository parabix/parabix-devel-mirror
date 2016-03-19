/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef DELETION_H
#define DELETION_H

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

std::vector<Value *> parallel_prefix_deletion_masks(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * del_mask);

// Applying Deletion Masks to a Stream

Value * apply_parallel_prefix_deletion(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * del_mask, std::vector<Value *> mv, Value * strm);


#endif

