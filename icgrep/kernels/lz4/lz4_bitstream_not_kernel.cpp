
#include "lz4_bitstream_not_kernel.h"

#include <kernels/kernel_builder.h>
#include <pablo/pabloAST.h>
#include <pablo/builder.hpp>
#include <pablo/pe_pack.h>

#include <llvm/Support/raw_ostream.h>


using namespace llvm;
using namespace kernel;
using namespace std;
using namespace pablo;


namespace kernel {
    LZ4BitStreamNotKernel::LZ4BitStreamNotKernel(const std::unique_ptr<pablo::PabloKernel::KernelBuilder> &b):
            PabloKernel(b, "LZ4BitStreamNotKernel",
                        {Binding{b->getStreamSetTy(1), "inputBitStream"}},
                        {Binding{b->getStreamSetTy(1), "outputBitStream"}}){
    }

    void LZ4BitStreamNotKernel::generatePabloMethod() {
        PabloBuilder pb(getEntryScope());

        Var *e0Marker = getInputStreamVar("inputBitStream");
        PabloAST *v = pb.createExtract(e0Marker, 0);

        Var *eMarker = getOutputStreamVar("outputBitStream");
        pb.createAssign(pb.createExtract(eMarker, 0), pb.createNot(v));
    }
}

