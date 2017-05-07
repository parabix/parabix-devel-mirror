/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STREAMSMERGE_H
#define STREAMSMERGE_H

#include "kernel.h"
namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StreamsMerge : public BlockOrientedKernel {
public:
    
    StreamsMerge(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamsPerSet=1, unsigned inputSets = 1);
    
protected:
    
    void generateDoBlockMethod() override;
    
private:
    const unsigned mStreamsPerSet;
    const unsigned mInputSets;
};

}
    
#endif

