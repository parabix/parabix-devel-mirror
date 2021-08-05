/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernel/core/kernel.h>
#include <kernel/core/streamset.h>
#include <toolchain/toolchain.h>
#include <kernel/core/kernel_builder.h>
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
using StreamSetPort = Kernel::StreamSetPort;
using AttrId = Attribute::KindId;
using RateId = ProcessingRate::KindId;
using Rational = ProcessingRate::Rational;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::generateKernelMethod(BuilderRef b) {
    generateMultiBlockLogic(b, b->getNumOfStrides());
}

// MULTI-BLOCK KERNEL CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(
    BuilderRef b,
    std::string && kernelName,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_parameters,
    Bindings && scalar_outputs,
    InternalScalars && internal_scalars)
: MultiBlockKernel(b,
    TypeId::MultiBlock,
    std::move(kernelName),
    std::move(stream_inputs),
    std::move(stream_outputs),
    std::move(scalar_parameters),
    std::move(scalar_outputs),
    std::move(internal_scalars)) {

}

MultiBlockKernel::MultiBlockKernel(BuilderRef b,
    const TypeId typeId,
    std::string && kernelName,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_parameters,
    Bindings && scalar_outputs,
    InternalScalars && internal_scalars)
: Kernel(b, typeId,
     std::move(kernelName),
     std::move(stream_inputs),
     std::move(stream_outputs),
     std::move(scalar_parameters),
     std::move(scalar_outputs),
     std::move(internal_scalars)) {

}

}
