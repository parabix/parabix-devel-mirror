#include <kernels/optimizationbranch.h>
#include "optimizationbranch_compiler.hpp"
#include <kernels/kernel_builder.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/container/flat_map.hpp>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

#warning at compilation, this must verify that the I/O rates of the branch permits the rates of the branches

#warning move most of this logic this into the optimizationbranch compiler

using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace kernel {

using AttrId = Attribute::KindId;

using ScalarDependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, Value *, unsigned>;
using ScalarVertex = ScalarDependencyGraph::vertex_descriptor;
using ScalarDependencyMap = flat_map<const Relationship *, ScalarVertex>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCompiler = llvm::make_unique<OptimizationBranchCompiler>(this);
    mCompiler->addBranchProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateKernelMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateFinalizeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
    mNonZeroKernel->addKernelDeclarations(b);
    mAllZeroKernel->addKernelDeclarations(b);
    Kernel::addKernelDeclarations(b);
}

OptimizationBranch::OptimizationBranch(const std::unique_ptr<KernelBuilder> & b,
    std::string && signature,
    not_null<Relationship *> condition,
    not_null<Kernel *> nonZeroKernel,
    not_null<Kernel *> allZeroKernel,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_inputs,
    Bindings && scalar_outputs)
: Kernel(b, TypeId::OptimizationBranch, std::move(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs),
         {})
, mCondition(condition.get())
, mNonZeroKernel(nonZeroKernel.get())
, mAllZeroKernel(allZeroKernel.get()) {

}

}
