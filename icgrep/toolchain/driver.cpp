#include "driver.h"
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <llvm/IR/Module.h>
#include <toolchain/toolchain.h>
#include <toolchain/object_cache.h>
#include <llvm/Support/raw_ostream.h>

using namespace kernel;

using RelationshipAllocator = Relationship::Allocator;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineWithIO
 ** ------------------------------------------------------------------------------------------------------------- */
std::unique_ptr<ProgramBuilder> BaseDriver::makePipelineWithIO(Bindings stream_inputs, Bindings stream_outputs, Bindings scalar_inputs, Bindings scalar_outputs) {
    return llvm::make_unique<ProgramBuilder>(*this, std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_inputs), std::move(scalar_outputs));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipeline
 ** ------------------------------------------------------------------------------------------------------------- */
std::unique_ptr<ProgramBuilder> BaseDriver::makePipeline(Bindings scalar_inputs, Bindings scalar_outputs) {
    return llvm::make_unique<ProgramBuilder>(*this, Bindings{}, Bindings{}, std::move(scalar_inputs), std::move(scalar_outputs));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateStreamSet
 ** ------------------------------------------------------------------------------------------------------------- */
StreamSet * BaseDriver::CreateStreamSet(const unsigned NumElements, const unsigned FieldWidth) {
    RelationshipAllocator A(mAllocator);
    return new (A) StreamSet(getContext(), NumElements, FieldWidth);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateConstant
 ** ------------------------------------------------------------------------------------------------------------- */
Scalar * BaseDriver::CreateScalar(llvm::Type * scalarType) {
    RelationshipAllocator A(mAllocator);
    return new (A) Scalar(scalarType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateConstant
 ** ------------------------------------------------------------------------------------------------------------- */
Scalar * BaseDriver::CreateConstant(llvm::Constant * const value) {
    RelationshipAllocator A(mAllocator);
    return new (A) ScalarConstant(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeCache
 ** ------------------------------------------------------------------------------------------------------------- */
void BaseDriver::addKernel(Kernel * const kernel) {
    if (ParabixObjectCache::checkForCachedKernel(iBuilder, kernel)) {
        assert (kernel->getModule());
        mCachedKernel.emplace_back(kernel);
    } else {
        if (kernel->getModule() == nullptr) {
            kernel->makeModule(iBuilder);
        }
        mUncachedKernel.emplace_back(kernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
BaseDriver::BaseDriver(std::string && moduleName)
: mContext(new llvm::LLVMContext())
, mMainModule(new llvm::Module(moduleName, *mContext))
, iBuilder(nullptr) {
    ParabixObjectCache::initializeCacheSystems();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deconstructor
 ** ------------------------------------------------------------------------------------------------------------- */
BaseDriver::~BaseDriver() {

}
