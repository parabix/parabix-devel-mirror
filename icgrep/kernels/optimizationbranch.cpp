#include "optimizationbranch.h"

#warning at compilation, this must verify that the I/O rates of the branch permits the rates of the branches

using namespace llvm;

namespace kernel {

void OptimizationBranch::linkExternalMethods(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->linkExternalMethods(b);
    mFalseKernel->linkExternalMethods(b);
}

void OptimizationBranch::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->generateInitializeMethod(b);
    mFalseKernel->generateInitializeMethod(b);
}

void OptimizationBranch::initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<llvm::Value *> & args) {

}

void OptimizationBranch::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

}

void OptimizationBranch::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->generateFinalizeMethod(b);
    mFalseKernel->generateFinalizeMethod(b);
}

void OptimizationBranch::addAdditionalFunctions(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->addAdditionalFunctions(b);
    mFalseKernel->addAdditionalFunctions(b);
}

Value * OptimizationBranch::finalizeInstance(const std::unique_ptr<KernelBuilder> & b) {

    // TODO: to have a returnable result here, we need to store the
    // scalars in this kernel or the pipeline.

//    Value * trueResult = mTrueKernel->finalizeInstance(b);
//    Value * falseResult = mFalseKernel->finalizeInstance(b);
    return nullptr;
}

void OptimizationBranch::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {

}

std::vector<Value *> OptimizationBranch::getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) {
    return std::vector<Value *>{};
}

OptimizationBranch::OptimizationBranch(
    std::string && signature,
    not_null<StreamSet *> condition,
    not_null<Kernel *> trueKernel,
    not_null<Kernel *> falseKernel,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_inputs,
    Bindings && scalar_outputs)
: Kernel(TypeId::OptimizationBranch, std::move(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs), {})
, mCondition(condition.get())
, mTrueKernel(trueKernel.get())
, mFalseKernel(falseKernel.get()) {


}

OptimizationBranch::~OptimizationBranch() {

}

}
