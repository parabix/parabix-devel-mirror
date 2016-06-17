/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PABLO_KERNEL_H
#define PABLO_KERNEL_H

#include <kernels/interface.h>
#include <kernels/streamset.h>
#include <IDISA/idisa_builder.h>
#include <pablo/function.h>

class PabloCompiler;

namespace pablo {

class PabloKernel : public KernelInterface {
public:
    PabloKernel(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    PabloFunction * function,
                    std::vector<std::string> accumulators);
// At present only population count accumulator are supported,
// using the pablo.Count operation.

    std::unique_ptr<llvm::Module> createKernelModule() override;
    
protected:
    // The default method for Pablo final block processing sets the
    // EOFmark bit and then calls the standard DoBlock function.
    // This may be overridden for specialized processing.
    virtual void addFinalBlockMethod(Module * m);
    
    PabloFunction * mPabloFunction;

    std::vector<ScalarBinding> accumBindings(std::vector<std::string> accum_names);

    friend class PabloCompiler;
};
}
#endif // PABLO_KERNEL_H
