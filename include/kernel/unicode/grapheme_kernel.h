/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GRAPHEME_KERNEL_H
#define GRAPHEME_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel {
    
class GraphemeClusterBreakKernel : public pablo::PabloKernel {
public:
    GraphemeClusterBreakKernel(BuilderRef kb, StreamSet * BasisBits, StreamSet * RequiredStreams, StreamSet * GCB_stream);
protected:
    void generatePabloMethod() override;
};

}
#endif
