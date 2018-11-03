/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/kernel.h>

#include <toolchain/toolchain.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <sstream>

using namespace llvm;

namespace kernel {

using Port = Kernel::Port;
using StreamPort = Kernel::StreamSetPort;
using AttrId = Attribute::KindId;
using RateId = ProcessingRate::KindId;
using RateValue = ProcessingRate::RateValue;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

    Value * const numOfStrides = b->CreateSelect(mIsFinal, b->getSize(1), mNumOfStrides);

    generateMultiBlockLogic(b, numOfStrides);

    BasicBlock * const exit = b->CreateBasicBlock();
    if (LLVM_UNLIKELY(hasAttribute(AttrId::CanTerminateEarly) || hasAttribute(AttrId::MustExplicitlyTerminate))) {
        Value * const terminated = b->getTerminationSignal();
        BasicBlock * const regularExit = b->CreateBasicBlock("regularExit", exit);
        b->CreateUnlikelyCondBr(terminated, exit, regularExit);

        b->SetInsertPoint(regularExit);
    }
    incrementItemCountsOfCountableRateStreams(b);
    b->CreateBr(exit);

    b->SetInsertPoint(exit);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief incrementItemCountsOfCountableRateStreams
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::incrementItemCountsOfCountableRateStreams(const std::unique_ptr<KernelBuilder> & b) {

    const auto numOfInputs = getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputStreamSetBinding(i);
        if (isCountable(input)) {
            Value * avail = getAvailableInputItems(i);
            b->setNonDeferredProcessedItemCount(input, avail);
        }
    }

    const auto numOfOutputs = getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputStreamSetBinding(i);
        if (isCountable(output)) {
            Value * avail = b->getCapacity(output.getName());
            b->setNonDeferredProducedItemCount(output, avail);
        }
    }
}

// MULTI-BLOCK KERNEL CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(std::string && kernelName,
                                   Bindings && stream_inputs,
                                   Bindings && stream_outputs,
                                   Bindings && scalar_parameters,
                                   Bindings && scalar_outputs,
                                   Bindings && internal_scalars)
: Kernel(std::move(kernelName),
         std::move(stream_inputs),
         std::move(stream_outputs),
         std::move(scalar_parameters),
         std::move(scalar_outputs),
         std::move(internal_scalars)) {

}

}
