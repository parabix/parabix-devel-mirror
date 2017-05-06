/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "match_count.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Value.h>

using namespace llvm;

namespace kernel {

MatchCount::MatchCount(const std::unique_ptr<IDISA::IDISA_Builder> & iBuilder)
    : BlockOrientedKernel("matchCount",
                        {Binding{iBuilder->getStreamSetTy(1, 1), "matches"}}, {}, {}, {Binding{iBuilder->getSizeTy(), "matchedLineCount"}}, {}) {
    }

void MatchCount::generateDoBlockMethod() {

    const unsigned counterSize = iBuilder->getSizeTy()->getBitWidth();
    Value * to_count = loadInputStreamBlock("matches", iBuilder->getInt32(0));
    Value * count = getScalarField("matchedLineCount");
    
    Value * value = nullptr;
    Value * const partial = iBuilder->simd_popcount(counterSize, to_count);
    if (LLVM_UNLIKELY(counterSize <= 1)) {
        value = partial;
    } else {
        value = iBuilder->mvmd_extract(counterSize, partial, 0);
        const auto fields = (iBuilder->getBitBlockWidth() / counterSize);
        for (unsigned i = 1; i < fields; ++i) {
            Value * temp = iBuilder->mvmd_extract(counterSize, partial, i);
            value = iBuilder->CreateAdd(value, temp);
        }
    }
    value = iBuilder->CreateAdd(value, count);
    setScalarField("matchedLineCount", value);
}


}
