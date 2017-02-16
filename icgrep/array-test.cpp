/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <IR_Gen/idisa_builder.h>                  // for IDISA_Builder
#include <IR_Gen/idisa_target.h>                   // for GetIDISA_Builder
#include <kernels/mmap_kernel.h>                   // for MMapSourceKernel
#include <kernels/s2p_kernel.h>                    // for S2PKernel
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
#include <iostream>
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
    Var * matches = kernel->getOutputStreamVar("matches");

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

    Var * const all_closed = pb.createVar("all_closed", closed);
    Var * const pending_lparen = pb.createVar("pending_lparen", pb.createAnd(pscan, lparen));
    PabloAST * unmatched_rparen = pb.createAnd(rparen, pb.createNot(closed));
    Var * const in_play = pb.createVar("in_play", pb.createOr(pending_lparen, unmatched_rparen));
    Var * const errors = pb.createVar("errors", pb.createZeroes());

    Integer * const one = pb.getInteger(1);
    Var * const index = pb.createVar("i", one);

    PabloBuilder body = PabloBuilder::Create(pb);

    pb.createWhile(pending_lparen, body);

        pscan = body.createScanTo(body.createAdvance(pending_lparen, 1), in_play);
        closed = body.createAnd(pscan, rparen);
        body.createAssign(body.createExtract(matches, index), closed);
        body.createAssign(all_closed, body.createOr(all_closed, closed));
        body.createAssign(errors, body.createOr(errors, body.createAtEOF(pscan)));

        body.createAssign(pending_lparen, body.createAnd(pscan, lparen));

        unmatched_rparen = body.createAnd(rparen, body.createNot(all_closed));
        body.createAssign(in_play, body.createOr(pending_lparen, unmatched_rparen));
        body.createAssign(index, body.createAdd(index, one));


    pb.createAssign(errors, pb.createOr(errors, pb.createAnd(rparen, pb.createNot(all_closed))));
    pb.createAssign(kernel->getOutputStreamVar("errors"), errors);

    pb.print(errs());

}

//42	def Match_Parens(lex, matches):
//43	        parens = lex.LParen | lex.RParen
//44	        i = 0
//45	        pscan = pablo.AdvanceThenScanTo(lex.LParen, parens)
//46	        matches.closed[0] = pscan & lex.RParen
//47	        all_closed = matches.closed[0]
//48	        matches.error = pablo.atEOF(pscan)
//49	        # Not matched, still pending.
//50	        pending_LParen = pscan & lex.LParen
//51	        RParen_unmatched = lex.RParen &~ matches.closed[0]
//52	        inPlay = pending_LParen | RParen_unmatched
//53	        while pending_LParen:
//54	                i += 1
//55	                pscan = pablo.AdvanceThenScanTo(pending_LParen, inPlay)
//56	                matches.closed[i] = pscan & lex.RParen
//57	                all_closed |= matches.closed[i]
//58	                matches.error |= pablo.atEOF(pscan)
//59	                pending_LParen = pscan & lex.LParen
//60	                RParen_unmatched = lex.RParen &~ all_closed
//61	                inPlay = pending_LParen | RParen_unmatched
//62	        #
//63	        # Any closing paren that was not actually used to close
//64	        # an opener is in error.
//65	        matches.error |= lex.RParen &~ all_closed

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
    ExpandableBuffer matches(iBuilder, iBuilder->getStreamSetTy(count, 1), 2);
    SingleBlockBuffer errors(iBuilder, iBuilder->getStreamTy());

    MMapSourceKernel mmapK(iBuilder); 
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});
    
    S2PKernel s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});

    PabloKernel bm(iBuilder, "MatchParens",
        {Binding{iBuilder->getStreamSetTy(8), "input"}},
        {Binding{iBuilder->getStreamSetTy(count), "matches"}, Binding{iBuilder->getStreamTy(), "errors"}});

    generate(&bm);

    bm.generateKernel({&BasisBits}, {&matches, &errors});

    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main, 0));

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    matches.allocateBuffer();
    errors.allocateBuffer();

    generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &bm});
    iBuilder->CreateRetVoid();

    return main;
}

typedef void (*MatchParens)(char * byteStream, size_t fileSize);

MatchParens generateAlgorithm() {
    LLVMContext ctx;
    Module * M = new Module("mp", ctx);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * f = pipeline(idb, 2);

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
