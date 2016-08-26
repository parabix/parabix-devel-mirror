/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PABLO_KERNEL_H
#define PABLO_KERNEL_H

#include <kernels/kernel.h>
#include <IDISA/idisa_builder.h>
#include <pablo/function.h>

namespace pablo {
    
class PabloCompiler; class CarryManager;


class PabloKernel : public kernel::KernelBuilder {
public:
    PabloKernel(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    PabloFunction * function,
                    std::vector<std::string> accumulators);
    // At present only population count accumulator are supported,
    // using the pablo.Count operation.
    
protected:
    // A custom method for preparing kernel declarations is needed,
    // so that the carry data requirements may be accommodated before
    // finalizing the KernelStateType.
    void prepareKernel() override;

    void generateDoBlockMethod() override;
    
    // The default method for Pablo final block processing sets the
    // EOFmark bit and then calls the standard DoBlock function.
    // This may be overridden for specialized processing.
    virtual void generateFinalBlockMethod() override;
    
    PabloFunction * mPabloFunction;

    std::vector<ScalarBinding> accumBindings(std::vector<std::string> accum_names);
    
    //std::unique_ptr<pablo::PabloCompiler> pablo_compiler;
    PabloCompiler * pablo_compiler;

    friend class PabloCompiler;
    friend class CarryManager;
};
}
#endif // PABLO_KERNEL_H
