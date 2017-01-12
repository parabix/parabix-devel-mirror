/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>


#include <toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include "llvm/Linker/Linker.h"
#include <llvm/IR/Verifier.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>

#include <pablo/pablo_kernel.h>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/streamset.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/mmap_kernel.h>
#include <kernels/pipeline.h>
#include <pablo/builder.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/s2p_kernel.h>
#include <kernels/stdout_kernel.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore);

using namespace pablo;
using namespace kernel;
using namespace parabix;

void generate(PabloKernel * kernel, const unsigned size) {

    PabloBuilder pb(kernel->getEntryBlock());

    Var * input = kernel->addInput("input", kernel->getStreamSetTy(8));
    Var * matches = kernel->addOutput("matches", kernel->getStreamSetTy(size));

    PabloAST * basis[8];
    for (int i = 0; i < 8; ++i) {
        basis[i] = pb.createExtract(input, i);
    }

    PabloAST * temp1 = pb.createOr(basis[0], basis[1]);
    PabloAST * temp2 = pb.createAnd(basis[2], pb.createNot(basis[3]));
    PabloAST * temp3 = pb.createAnd(temp2, pb.createNot(temp1));
    PabloAST * temp4 = pb.createAnd(basis[4], pb.createNot(basis[5]));
    PabloAST * temp5 = pb.createOr(basis[6], basis[7]);
    PabloAST * temp6 = pb.createAnd(temp4, pb.createNot(temp5));
    PabloAST * lparen = pb.createAnd(temp3, temp6, "lparens");
    PabloAST * temp7 = pb.createAnd(basis[7], pb.createNot(basis[6]));
    PabloAST * temp8 = pb.createAnd(temp4, temp7);
    PabloAST * rparen = pb.createAnd(temp3, temp8, "rparens");
    PabloAST * parens = pb.createOr(lparen, rparen);

    PabloAST * pscan = pb.createScanTo(pb.createAdvance(lparen, 1), parens, "pscan");

    PabloAST * closed = pb.createAnd(pscan, rparen, "closed");

    pb.createAssign(pb.createExtract(matches, 0), closed);

    Var * all_closed = pb.createVar("all_closed", closed);
    Var * pending_lparen = pb.createVar("pending_lparen", pb.createAnd(pscan, lparen));
    Var * unmatched_rparen = pb.createVar("unmatched_rparen", pb.createAnd(rparen, pb.createNot(closed)));
    Var * in_play = pb.createVar("in_play", pb.createOr(pending_lparen, unmatched_rparen));


    Integer * one = pb.getInteger(1);

    Var * index = pb.createVar("i", one);

    PabloBuilder body = PabloBuilder::Create(pb);

    pb.createWhile(pending_lparen, body);

        pscan = body.createScanTo(body.createAdvance(pending_lparen, 1), in_play);
        closed = body.createAnd(pscan, rparen);
        body.createAssign(body.createExtract(matches, index), closed);
        body.createAssign(pending_lparen, body.createAnd(pscan, lparen));
        body.createAssign(all_closed, body.createOr(all_closed, closed));
        body.createAssign(unmatched_rparen, body.createAnd(rparen, body.createNot(all_closed)));
        body.createAssign(in_play, body.createOr(pending_lparen, unmatched_rparen));
        body.createAssign(index, body.createAdd(index, one));


    pb.print(errs());

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
    
    ExternalFileBuffer ByteStream(iBuilder, byteStreamTy);
    SingleBlockBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8, 1));
    SingleBlockBuffer matches(iBuilder, iBuilder->getStreamSetTy(count, 1));

    MMapSourceKernel mmapK(iBuilder); 
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});
    
    S2PKernel  s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});

    PabloKernel bm(iBuilder, "MatchParens");
    generate(&bm, count);

    bm.generateKernel({&BasisBits}, {&matches});

    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main, 0));

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    matches.allocateBuffer();

    generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &bm});
    iBuilder->CreateRetVoid();

    return main;
}

typedef void (*MatchParens)(char * byteStream, size_t fileSize);

MatchParens generateAlgorithm() {
    LLVMContext ctx;
    Module * M = new Module("mp", ctx);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * f = pipeline(idb, 10);

    verifyModule(*M, &dbgs());

    ExecutionEngine * wcEngine = JIT_to_ExecutionEngine(M);

    wcEngine->finalizeObject();

    delete idb;

    return reinterpret_cast<MatchParens>(wcEngine->getPointerToFunction(f));
}

void run(MatchParens f, const std::string & fileName) {
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
            f(fileBuffer, fileSize);
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
