
#include "lz4_not_kernel.h"
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>

using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

LZ4NotKernel::LZ4NotKernel(const std::unique_ptr<kernel::KernelBuilder> &kb):
        PabloKernel(kb,
                    "LZ4NotKernel",
                    {Binding{kb->getStreamSetTy(1, 1), "inputStream"}},
                    {Binding{kb->getStreamSetTy(1, 1), "outputStream"}})
{

}

void LZ4NotKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * i = pb.createExtract(getInputStreamVar("inputStream"), pb.getInteger(0));
    PabloAST * o = pb.createNot(i);
    pb.createAssign(
            pb.createExtract(getOutputStreamVar("outputStream"), pb.getInteger(0)),
            o
    );
}
