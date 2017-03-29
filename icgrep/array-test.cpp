/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <IR_Gen/idisa_builder.h>                  // for IDISA_Builder
#include <IR_Gen/idisa_target.h>                   // for GetIDISA_Builder
#include <kernels/mmap_kernel.h>                   // for MMapSourceKernel
#include <kernels/s2p_kernel.h>                    // for S2PKernel
#include <kernels/alignedprint.h>
#include <kernels/streamset.h>                     // for SingleBlockBuffer
#include <kernels/pipeline.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for ExecutionEngine
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/IR/Verifier.h>                      // for verifyModule
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/raw_ostream.h>              // for errs
#include <pablo/pablo_kernel.h>                    // for PabloKernel
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <toolchain.h>                             // for JIT_to_ExecutionEn...
#include <pablo/builder.hpp>                       // for PabloBuilder
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include "llvm/ADT/StringRef.h"                    // for StringRef
#include "llvm/IR/CallingConv.h"                   // for ::C
#include "llvm/IR/DerivedTypes.h"                  // for ArrayType
#include "llvm/IR/LLVMContext.h"                   // for LLVMContext
#include "llvm/IR/Value.h"                         // for Value
#include "llvm/Support/Debug.h"                    // for dbgs
#include <pablo/pablo_toolchain.h>
#include <iostream>

#include <pablo/passes/ssapass.h>

namespace llvm { class Type; }
namespace pablo { class Integer; }
namespace pablo { class Var; }

using namespace pablo;
using namespace kernel;
using namespace parabix;
using namespace llvm;

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore);

void generate(PabloKernel * kernel) {

    PabloBuilder pb(kernel->getEntryBlock());

    Var * input = kernel->getInputStreamVar("input");

    PabloAST * basis[8];
    for (int i = 0; i < 8; ++i) {
        basis[i] = pb.createExtract(input, i);
    }

    PabloAST * temp1 = pb.createOr(basis[0], basis[1], "temp1");
    PabloAST * temp2 = pb.createAnd(basis[2], pb.createNot(basis[3]), "temp2");
    PabloAST * temp3 = pb.createAnd(temp2, pb.createNot(temp1), "temp3");
    PabloAST * temp4 = pb.createAnd(basis[4], pb.createNot(basis[5]), "temp4");
    PabloAST * temp5 = pb.createOr(basis[6], basis[7], "temp5");
    PabloAST * temp6 = pb.createAnd(temp4, pb.createNot(temp5), "temp6");
    PabloAST * lparen = pb.createAnd(temp3, temp6, "lparens");
    PabloAST * temp7 = pb.createAnd(basis[7], pb.createNot(basis[6]), "temp7");
    PabloAST * temp8 = pb.createAnd(temp4, temp7, "temp8");
    PabloAST * rparen = pb.createAnd(temp3, temp8, "rparens");
    PabloAST * parens = pb.createOr(lparen, rparen, "parens");


    Var * const pending_lparen = pb.createVar("pending_lparen", lparen);
    Var * const all_closed = pb.createVar("all_closed", pb.createZeroes());
    Var * const accumulated_errors = pb.createVar("accumulated_errors", pb.createZeroes());
    Var * const in_play = pb.createVar("in_play", parens);
    Var * const index = pb.createVar("i", pb.getInteger(0));

    Var * matches = kernel->getOutputStreamVar("matches");

//    PabloBuilder outer = PabloBuilder::Create(pb);

//    pb.createWhile(pending_lparen, outer);

    PabloBuilder body = PabloBuilder::Create(pb); // outer

    pb.createWhile(pending_lparen, body); // outer

        PabloAST * adv_pending_lparen = body.createAdvance(pending_lparen, 1);

        Var * pscan = body.createVar("pscan", pb.createZeroes());

        PabloBuilder ifPScan = PabloBuilder::Create(body);

        body.createIf(adv_pending_lparen, ifPScan); // <-- inefficient but tests whether we're probably calculating the summary later

            ifPScan.createAssign(pscan, ifPScan.createScanTo(adv_pending_lparen, in_play));

//        body.createAssign(pscan, body.createScanTo(adv_pending_lparen, in_play));

        body.createAssign(pending_lparen, body.createAnd(pscan, lparen));

        PabloAST * closed_rparen = body.createAnd(pscan, rparen, "closed_rparen");

        body.createAssign(all_closed, body.createOr(all_closed, closed_rparen));

        // Mark any opening paren without a matching closer as an error.
        PabloAST * unmatched_lparen = body.createAtEOF(pscan, "unmatched_lparen");

        body.createAssign(accumulated_errors, body.createOr(accumulated_errors, unmatched_lparen));

        PabloAST * pending_rparen = body.createAnd(rparen, body.createNot(all_closed, "open_rparen"), "pending_rparen");

        body.createAssign(in_play, body.createOr(pending_lparen, pending_rparen));

        body.createAssign(body.createExtract(matches, index), closed_rparen);

        body.createAssign(index, body.createAdd(index, body.getInteger(1)));

    // Mark any closing paren that was not actually used to close an opener as an error.
    PabloAST * const unmatched_rparen = pb.createAnd(rparen, pb.createNot(all_closed), "unmatched_rparen");
    pb.createAssign(kernel->getOutputStreamVar("errors"), pb.createOr(accumulated_errors, unmatched_rparen));

}

Function * pipeline(IDISA::IDISA_Builder * iBuilder, const unsigned count) {

    Type * byteStreamTy = iBuilder->getStreamSetTy(1, 8);

    Module * const mod = iBuilder->getModule();

    Function * const main = cast<Function>(mod->getOrInsertFunction("Main", iBuilder->getVoidTy(), byteStreamTy->getPointerTo(), iBuilder->getSizeTy(), nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;
    
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));

    MMapSourceKernel mmapK(iBuilder, segmentSize);
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});

    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);

    S2PKernel  s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});

    PabloKernel bm(iBuilder, "MatchParens",
        {Binding{iBuilder->getStreamSetTy(8), "input"}},
        {Binding{iBuilder->getStreamSetTy(count), "matches"}, Binding{iBuilder->getStreamTy(), "errors"}});

    generate(&bm);
//    SSAPass::transform(&bm);

//    pablo_function_passes(&bm);

    bm.getEntryBlock()->print(errs());

    ExpandableBuffer matches(iBuilder, iBuilder->getStreamSetTy(count), segmentSize * bufferSegments);
    SingleBlockBuffer errors(iBuilder, iBuilder->getStreamTy());

    bm.generateKernel({&BasisBits}, {&matches, &errors});

    PrintStreamSet printer(iBuilder, {"matches", "errors"});
    printer.generateKernel({&matches, &errors}, {});

    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main, 0));

    ByteStream.setStreamSetBuffer(inputStream);
    BasisBits.allocateBuffer();
    matches.allocateBuffer();
    errors.allocateBuffer();

//    generatePipeline(iBuilder, {&mmapK, &s2pk, &bm});

    generatePipeline(iBuilder, {&mmapK, &s2pk, &bm, &printer});
    iBuilder->CreateRetVoid();

    return main;
}

typedef void (*MatchParens)(char * byteStream, size_t fileSize);

MatchParens generateAlgorithm() {
    LLVMContext ctx;
    Module * M = new Module("mp", ctx);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * f = pipeline(idb, 3);

    verifyModule(*M, &dbgs());

    ExecutionEngine * wcEngine = JIT_to_ExecutionEngine(M);

    wcEngine->finalizeObject();

    delete idb;

    return reinterpret_cast<MatchParens>(wcEngine->getPointerToFunction(f));
}

template <typename T>
std::ostream & operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

void check(const char * byteStream, const size_t fileSize) {

    std::vector<size_t> unmatched_left;
    std::vector<size_t> unmatched_right;
    size_t maxDepth = 0;
    for (size_t i = 0; i < fileSize; ++i) {
        switch (byteStream[i]) {
            case '(':
                unmatched_left.push_back(i);
                maxDepth = std::max<size_t>(maxDepth, unmatched_left.size());
                break;
            case ')':
                if (unmatched_left.empty()) {
                    unmatched_right.push_back(i);
                } else {
                    unmatched_left.pop_back();
                }
            default:
                break;
        }
    }

    std::cerr << "maximum depth:        " << maxDepth << "\n"
                 "invalid left parens:  " << unmatched_left << "\n"
                 "invalid right parens: " << unmatched_right <<
                 std::endl;
}

void run(MatchParens match, const std::string & fileName) {
    const boost::filesystem::path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
        size_t fileSize = file_size(file);
        boost::iostreams::mapped_file_source mappedFile;
        if (fileSize > 0) {
            mappedFile.open(fileName);
            char * fileBuffer = const_cast<char *>(mappedFile.data());
            check(fileBuffer, fileSize);
            match(fileBuffer, fileSize);
            mappedFile.close();
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
    }
}

int main(int argc, char *argv[]) {
    cl::ParseCommandLineOptions(argc, argv);
    auto f = generateAlgorithm();
    for (const auto & inputFile : inputFiles) {
        run(f, inputFile);
    }
    return 0;
}
