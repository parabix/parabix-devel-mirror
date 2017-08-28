#include "driver.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <kernels/kernel.h>
#include <llvm/IR/Module.h>

using namespace llvm;
using namespace parabix;

using Kernel = kernel::Kernel;
using KernelBuilder = kernel::KernelBuilder;

Driver::Driver(std::string && moduleName)
: mContext(new llvm::LLVMContext())
, mMainModule(new Module(moduleName, *mContext))
, iBuilder(nullptr) {

}

ExternalBuffer * Driver::addExternalBuffer(std::unique_ptr<ExternalBuffer> b) {
    mOwnedBuffers.emplace_back(std::move(b));
    return cast<ExternalBuffer>(mOwnedBuffers.back().get());
}

StreamSetBuffer * Driver::addBuffer(std::unique_ptr<StreamSetBuffer> b) {
    b->allocateBuffer(iBuilder);
    mOwnedBuffers.emplace_back(std::move(b));
    return mOwnedBuffers.back().get();
}

kernel::Kernel * Driver::addKernelInstance(std::unique_ptr<kernel::Kernel> kb) {
    mOwnedKernels.emplace_back(std::move(kb));
    return mOwnedKernels.back().get();
}

void Driver::deallocateBuffers() {
    for (auto &b : mOwnedBuffers) {
        b->releaseBuffer(iBuilder);
    }
}
