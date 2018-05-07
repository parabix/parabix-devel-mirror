
#include <pablo/builder.hpp>
#include "lz4_generate_deposit_stream.h"
#include <kernels/kernel_builder.h>

using namespace llvm;
using namespace kernel;
using namespace std;
using namespace pablo;

namespace kernel {
    void LZ4GenerateDepositStreamKernel::generatePabloMethod() {
        PabloBuilder pb(getEntryScope());

        Var* m0Marker = getInputStreamVar("m0");
        PabloAST* v = pb.createExtract(m0Marker, 0);

        Var* dMarker = getOutputStreamVar("deposit");

        // d_marker = ~(M0_marker | pablo.Advance(M0_marker))
        PabloAST* value = pb.createAdvance(v, 1);
        value = pb.createOr(v, value);
        value = pb.createNot(value);

        pb.createAssign(pb.createExtract(dMarker, 0), value);
    }

    LZ4GenerateDepositStreamKernel::LZ4GenerateDepositStreamKernel (const std::unique_ptr<kernel::KernelBuilder> & b)
            :
            PabloKernel(
                    b,
                    "lz4_genereate_deposit_stream_kernel",
                    {// Stream Input
                            {b->getStreamSetTy(1), "m0"}
                    },
                    {// Stream Output
                            {b->getStreamSetTy(1), "deposit"}
                    }){

    }
}
