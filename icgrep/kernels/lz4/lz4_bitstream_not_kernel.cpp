
#include "lz4_bitstream_not_kernel.h"
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>


using namespace llvm;
using namespace kernel;
using namespace std;
using namespace pablo;

namespace kernel {

    void LZ4BitStreamNotKernel::generatePabloMethod() {
        PabloBuilder pb(getEntryScope());

        Var *e0Marker = getInputStreamVar("e1");
        PabloAST *v = pb.createExtract(e0Marker, 0);

        Var *eMarker = getOutputStreamVar("e");
        pb.createAssign(pb.createExtract(eMarker, 0), pb.createNot(v));
    }

    LZ4BitStreamNotKernel::LZ4BitStreamNotKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
            :
            PabloKernel(
                    b,
                    "lz4_e1_to_e_kernel",
                    {// Stream Input
                            {b->getStreamSetTy(1), "e1"}
                    },
                    {// Stream Output
                            {b->getStreamSetTy(1), "e"}
                    }
            ) {
    }
}