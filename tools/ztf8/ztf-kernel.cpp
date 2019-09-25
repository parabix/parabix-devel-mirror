/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "ztf-kernel.h"
#include <re/adt/re_name.h>
#include <re/adt/re_re.h>
#include <pablo/bixnum/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>

using namespace pablo;
using namespace kernel;

WordMarkKernel::WordMarkKernel(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * WordMarks)
: PabloKernel(kb, "WordMarks", {Binding{"source", BasisBits}}, {Binding{"WordMarks", WordMarks}}) { }

void WordMarkKernel::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("source"));
    UCD::UCDCompiler ucdCompiler(ccc);
    re::Name * word = re::makeName("word", re::Name::Type::UnicodeProperty);
    word = llvm::cast<re::Name>(re::resolveUnicodeNames(word));
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(word, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(word);
    if (f == nameMap.end()) llvm::report_fatal_error("Cannot find word property");
    PabloAST * wordChar = f->second;
    pb.createAssign(pb.createExtract(getOutputStreamVar("WordMarks"), pb.getInteger(0)), wordChar);
}

void ByteRun::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * excluded = getInputStreamSet("excluded")[0];

    PabloAST * mismatches = pb.createZeroes();
    for (unsigned i = 0; i < 8; i++) {
        mismatches = pb.createOr(mismatches,
                                 pb.createXor(basis[i], pb.createAdvance(basis[i], 1)),
                                 "mismatches_to_bit" + std::to_string(i));
    }
    PabloAST * matchesprior = pb.createInFile(pb.createNot(mismatches), "matchesprior");
    matchesprior = pb.createAnd(matchesprior, pb.createNot(excluded));
    pb.createAssign(pb.createExtract(getOutputStreamVar("runMask"), pb.getInteger(0)), matchesprior);
}

void ZTF_Codes::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    std::unique_ptr<cc::CC_Compiler> ccc;
    ccc = llvm::make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), basis);
    PabloAST * ASCII_lookahead = pb.createNot(pb.createLookahead(basis[7], 1));
    PabloAST * dictSym = pb.createAnd(ccc->compileCC(re::makeByte(0xC2, 0xDF)), ASCII_lookahead, "dictSym");
    PabloAST * byteRunSym = ccc->compileCC(re::makeByte(0xF9, 0xFF));
    pb.createAssign(pb.createExtract(getOutputStreamVar("byteRunCodes"), pb.getInteger(0)), byteRunSym);
    pb.createAssign(pb.createExtract(getOutputStreamVar("dictSymCodes"), pb.getInteger(0)), dictSym);
}

void ZTF_ExpansionDecoder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * byteRunCodes = getInputStreamSet("byteRunCodes")[0];
    PabloAST * dictSymCodes = getInputStreamSet("dictSymCodes")[0];
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    Var * lengthVar = getOutputStreamVar("insertBixNum");
    for (unsigned i = 0; i < 4; i++) {
        PabloAST * bit = pb.createAnd(dictSymCodes, basis[i+1]);
        if (i < 3) {
            bit = pb.createOr(bit, pb.createAnd(byteRunCodes, basis[i]));
        }
        pb.createAssign(pb.createExtract(lengthVar, pb.getInteger(i)), bit);
    }
    PabloAST * byteRunStart = pb.createAnd(pb.createLookahead(byteRunCodes, 1), pb.createNot(byteRunCodes), "byteRunStart");
    pb.createAssign(pb.createExtract(getOutputStreamVar("byteRunStart"), pb.getInteger(0)), byteRunStart);
}

void ZTF_Byte_Run_Decompression::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * runStart = getInputStreamSet("byteRunStarts")[0];
    PabloAST * spreadMask = getInputStreamSet("runSpreadMask")[0];
    std::vector<PabloAST *> ztf_u8_indexed = getInputStreamSet("ztf_u8_indexed");
    PabloAST * runMask = pb.createNot(spreadMask, "runMask");
    PabloAST * extendedMask = pb.createOr(runMask, pb.createAdvance(runMask, 1));
    Var * u8basis = getOutputStreamVar("u8output");
    for (unsigned i = 0; i < 8; i++) {
        PabloAST * bitMove = pb.createAdvance(pb.createAnd(ztf_u8_indexed[i], runStart), 1);
        PabloAST * bitCopy = pb.createMatchStar(bitMove, extendedMask);
        PabloAST * resultBit = pb.createSel(extendedMask, bitCopy, ztf_u8_indexed[i]);
        pb.createAssign(pb.createExtract(u8basis, pb.getInteger(i)), resultBit);
    }
}

