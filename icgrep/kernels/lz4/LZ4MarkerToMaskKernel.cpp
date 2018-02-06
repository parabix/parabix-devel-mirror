//
// Created by wxy325 on 2017/8/15.
//

#include "LZ4MarkerToMaskKernel.h"
#include <pablo/builder.hpp>
#include <kernels/kernel_builder.h>

using namespace llvm;
using namespace kernel;
using namespace std;
using namespace pablo;

namespace kernel {
    void LZ4MarkerToMaskKernel::generatePabloMethod() {
        PabloBuilder pb(getEntryScope());

        Var* startMarker = getInputStreamVar("startMarker");
        PabloAST* startMarkerV = pb.createExtract(startMarker, 0);

        Var* endMarker = getInputStreamVar("endMarker");
        PabloAST* endMarkerV = pb.createExtract(endMarker, 0);

        PabloAST* value = pb.createSubtract(endMarkerV, startMarkerV);
//        PabloAST* value = pb.createAdvance(endMarkerV, 1);

        Var* outputMask = getOutputStreamVar("outputMask");
        pb.createAssign(pb.createExtract(outputMask, 0), value);
    }

    LZ4MarkerToMaskKernel::LZ4MarkerToMaskKernel (std::string kernelName, const std::unique_ptr<kernel::KernelBuilder> & b)
            :
            PabloKernel(
                    b,
                    string(kernelName),
                    {// Stream Input
                            {b->getStreamSetTy(1), "startMarker"},
                            {b->getStreamSetTy(1), "endMarker"}
                    },
                    {// Stream Output
                            {b->getStreamSetTy(1), "outputMask"}
                    }){

    }
}
