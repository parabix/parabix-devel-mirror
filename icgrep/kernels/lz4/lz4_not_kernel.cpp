
#include "lz4_not_kernel.h"
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>

using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

LZ4NotKernel::LZ4NotKernel(const std::unique_ptr<kernel::KernelBuilder> &kb, StreamSet * inputStream, StreamSet * invertedOutputStream)
: PabloKernel(kb, "LZ4NotKernel", {Binding{"inputStream", inputStream}}, {Binding{"outputStream", invertedOutputStream}}) {

}

void LZ4NotKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    Integer * ZERO = pb.getInteger(0);
    PabloAST * input = pb.createExtract(getInputStreamVar("inputStream"), ZERO);
    PabloAST * inverted = pb.createNot(input);
    pb.createAssign(pb.createExtract(getOutputStreamVar("outputStream"), ZERO), inverted);
}
