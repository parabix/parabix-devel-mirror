/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef MATCHCOUNT_H
#define MATCHCOUNT_H

#include "kernel.h"
namespace IDISA { class IDISA_Builder; }

namespace kernel {

class MatchCount : public BlockOrientedKernel {
public:   
    MatchCount(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);   
protected:    
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

}
    
#endif

