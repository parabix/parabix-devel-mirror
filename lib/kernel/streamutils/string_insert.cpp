/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/streamutils/string_insert.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_var.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_lookahead.h>
#include <pablo/bixnum/bixnum.h>
#include <re/cc/cc_compiler_target.h>
#include <re/cc/cc_compiler.h>
#include <re/alphabet/alphabet.h>
#include <llvm/Support/Compiler.h>

using namespace pablo;
using namespace llvm;

namespace kernel {

LLVM_READONLY std::string StringInsertName(const std::vector<std::string> & insertStrs, const StreamSet * insertMarks) {
    std::string name = "StringInsertBixNum";
    for (auto s : insertStrs) {
        name += "_" + std::to_string(s.size());
    }
    if (insertMarks->getNumElements() < insertStrs.size()) {
        name += "_multiplexed";
    }
    return name;
}
    
StringInsertBixNum::StringInsertBixNum(BuilderRef b, const std::vector<std::string> & insertStrs,
                                       StreamSet * insertMarks, StreamSet * insertBixNum)
: PabloKernel(b, "StringInsertBixNum" + Kernel::getStringHash(StringInsertName(insertStrs, insertMarks)),
              {Binding{"insertMarks", insertMarks}},
              {Binding{"insertBixNum", insertBixNum}})
, mInsertStrings(insertStrs)
, mMultiplexing(insertMarks->getNumElements() < insertStrs.size())
, mBixNumBits(insertBixNum->getNumElements())
, mSignature(StringInsertName(insertStrs, insertMarks)) {

}

void StringInsertBixNum::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    std::vector<PabloAST *> insertMarks = getInputStreamSet("insertMarks");
    std::vector<PabloAST *> bixNum(mBixNumBits, pb.createZeroes());
    Var * insertVar = getOutputStreamVar("insertBixNum");
    for (unsigned i = 0; i < mInsertStrings.size(); i++) {
        PabloAST * stringMarks = mMultiplexing ? bnc.EQ(insertMarks, i) : insertMarks[i];
        for (unsigned j = 0; j < mBixNumBits; j++) {
            if ((mInsertStrings[i].size() >> j) & 1) {
                bixNum[j] = pb.createOr(bixNum[j], stringMarks);
            }
        }
    }
    for (unsigned j = 0; j < mBixNumBits; j++) {
        pb.createAssign(pb.createExtract(insertVar, pb.getInteger(j)), bixNum[j]);
    }
}

LLVM_READONLY std::string StringReplaceName(const std::vector<std::string> & insertStrs, const StreamSet * insertMarks) {
    std::string name = "StringReplaceBixNum";
    for (const auto & s : insertStrs) {
        name += "_" + s;
    }
    if (insertMarks->getNumElements() < insertStrs.size()) {
        name += "_multiplexed";
    }
    return name;
}

StringReplaceKernel::StringReplaceKernel(BuilderRef b, const std::vector<std::string> & insertStrs,
                                         StreamSet * basis, StreamSet * spreadMask,
                                         StreamSet * insertMarks, StreamSet * runIndex,
                                         StreamSet * output)
: PabloKernel(b, "StringReplaceBixNum" + Kernel::getStringHash(StringReplaceName(insertStrs, insertMarks)),
             {Binding{"basis", basis}, Binding{"spreadMask", spreadMask},
              Binding{"insertMarks", insertMarks, FixedRate(1), LookAhead(1 << (runIndex->getNumElements()))},
              Binding{"runIndex", runIndex}},
             {Binding{"output", output}})
, mInsertStrings(insertStrs)
, mMultiplexing(insertMarks->getNumElements() < insertStrs.size())
, mSignature(StringReplaceName(insertStrs, insertMarks)) {

}

void StringReplaceKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * spreadMask = getInputStreamSet("spreadMask")[0];
    std::vector<PabloAST *> insertMarks = getInputStreamSet("insertMarks");
    std::vector<PabloAST *> runIndex = getInputStreamSet("runIndex");
    Var * output = getOutputStreamVar("output");
    std::unique_ptr<cc::CC_Compiler> ccc;
    ccc = std::make_unique<cc::Parabix_CC_Compiler_Builder>(pb.getPabloBlock(), runIndex);
    PabloAST * runMask = pb.createInFile(pb.createNot(spreadMask));
    std::vector<PabloAST *> insertSpans(insertMarks.size());
    for (unsigned i = 0; i < mInsertStrings.size(); i++) {
        PabloAST * stringStart = pb.createLookahead(insertMarks[i], mInsertStrings[i].size());
        PabloAST * span = pb.createMatchStar(stringStart, runMask);
        insertSpans[i] = pb.createAnd(span, runMask);
    }
    for (unsigned bit = 0; bit < 8; bit++) {
        PabloAST * updated = basis[bit];
        for (unsigned i = 0; i < mInsertStrings.size(); i++) {
            PabloAST * stringMarks = mMultiplexing ? bnc.EQ(insertSpans, i) : insertSpans[i];
            re::CC * bitCC = re::makeCC(&cc::Byte);
            for (unsigned j = 0; j < mInsertStrings[i].size(); j++) {
                if ((mInsertStrings[i][j] >> bit) & 1) {
                    bitCC->insert(j);
                }
            }
            PabloAST * ccStrm = ccc->compileCC(bitCC);
            updated = pb.createSel(stringMarks, ccStrm, updated);
        }
        pb.createAssign(pb.createExtract(output, pb.getInteger(bit)), updated);
    }
}
}
