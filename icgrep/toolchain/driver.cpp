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

void Driver::deallocateBuffers() {
    for (const auto & b : mOwnedBuffers) {
        b->releaseBuffer(iBuilder);
    }
}
