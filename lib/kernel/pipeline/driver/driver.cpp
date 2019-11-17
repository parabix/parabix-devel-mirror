#include <kernel/pipeline/driver/driver.h>

#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <llvm/IR/Module.h>
#include <toolchain/toolchain.h>
#include <objcache/object_cache.h>
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
Scalar * BaseDriver::CreateScalar(not_null<llvm::Type *> scalarType) {
    RelationshipAllocator A(mAllocator);
    return new (A) Scalar(scalarType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateConstant
 ** ------------------------------------------------------------------------------------------------------------- */
Scalar * BaseDriver::CreateConstant(not_null<llvm::Constant *> value) {
    RelationshipAllocator A(mAllocator);
    return new (A) ScalarConstant(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void BaseDriver::addKernel(not_null<Kernel *> kernel) {

    // Verify the I/O relationships were properly set / defaulted in.

    for (Binding & input : kernel->getInputScalarBindings()) {
        if (input.getRelationship() == nullptr) {
            input.setRelationship(CreateScalar(input.getType()));
        }
    }
    for (Binding & input : kernel->getInputStreamSetBindings()) {
        if (LLVM_UNLIKELY(input.getRelationship() == nullptr)) {
            llvm::report_fatal_error(kernel->getName()+ "." + input.getName() + " must be set upon construction");
        }
    }
    for (Binding & output : kernel->getOutputStreamSetBindings()) {
        if (LLVM_UNLIKELY(output.getRelationship() == nullptr)) {
            llvm::report_fatal_error(kernel->getName()+ "." + output.getName() + " must be set upon construction");
        }
    }
    for (Binding & output : kernel->getOutputScalarBindings()) {
        if (output.getRelationship() == nullptr) {
            output.setRelationship(CreateScalar(output.getType()));
        }
    }

    // If the pipeline contained a single kernel, the pipeline builder will return the
    // original kernel. However, the module for that kernel is already owned by the JIT
    // engine so we avoid declaring it as cached or uncached.
    if (LLVM_LIKELY(kernel->getModule() == nullptr)) {
        if (LLVM_LIKELY(ParabixObjectCache::checkForCachedKernel(iBuilder, kernel))) {
            mCachedKernel.emplace_back(kernel);
        } else {
            if (kernel->getModule() == nullptr) {
                kernel->makeModule(iBuilder);
            }
            mUncachedKernel.emplace_back(kernel);
        }
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
