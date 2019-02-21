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

using PortType = Kernel::PortType;
using StreamPort = Kernel::StreamSetPort;
using AttrId = Attribute::KindId;
using RateId = ProcessingRate::KindId;
using RateValue = ProcessingRate::RateValue;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    generateMultiBlockLogic(b, b->CreateSelect(mIsFinal, b->getSize(1), mNumOfStrides));
}

// MULTI-BLOCK KERNEL CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(
    const std::unique_ptr<KernelBuilder> &b,
    std::string && kernelName,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_parameters,
    Bindings && scalar_outputs,
    Bindings && internal_scalars)
: MultiBlockKernel(b,
    TypeId::MultiBlock,
    std::move(kernelName),
    std::move(stream_inputs),
    std::move(stream_outputs),
    std::move(scalar_parameters),
    std::move(scalar_outputs),
    std::move(internal_scalars)) {

}

MultiBlockKernel::MultiBlockKernel(const std::unique_ptr<KernelBuilder> &b,
    const TypeId typeId,
    std::string && kernelName,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_parameters,
    Bindings && scalar_outputs,
    Bindings && internal_scalars)
: Kernel(b, typeId,
     std::move(kernelName),
     std::move(stream_inputs),
     std::move(stream_outputs),
     std::move(scalar_parameters),
     std::move(scalar_outputs),
     std::move(internal_scalars)) {

}

}
