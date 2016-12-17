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
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <kernels/streamset.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/pipeline.h>
#include <pablo/builder.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/printer_pablos.h>

#include <llvm/Support/raw_os_ostream.h>

using namespace pablo;
using namespace kernel;
using namespace parabix;

//
//

void generate(PabloKernel * kernel, const unsigned size) {

    PabloBuilder pb(kernel->getEntryBlock());

    Var * input = kernel->addInput("input", kernel->getStreamSetTy(size));
    Var * output = kernel->addOutput("output", kernel->getStreamSetTy(size));
    Var * index = pb.createVar("idx", pb.getInteger(0));

    Integer * one = pb.getInteger(1);

    PabloBuilder body = PabloBuilder::Create(pb);

    pb.createWhile(pb.createLessThan(index, pb.getInteger(size)), body);

        PabloAST * adv = body.createAdvance(body.createExtract(input, index), one, "adv");
        body.createAssign(body.createExtract(output, index), adv);

        body.createAssign(index, body.createAdd(index, one));



}

Function * pipeline(IDISA::IDISA_Builder * iBuilder, const unsigned count = 10) {

    PabloKernel main(iBuilder, "wc");
    generate(&main, count);

    ExternalFileBuffer input(iBuilder, iBuilder->getStreamSetTy(count));

    SingleBlockBuffer output(iBuilder, iBuilder->getStreamSetTy(count));

    main.generateKernel({&input}, {&output});

    return nullptr;
}


static ExecutionEngine * wcEngine = nullptr;

void * arrayTest() {
    Module * M = new Module("wc", getGlobalContext());
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * main_IR = pipeline(idb);

    verifyModule(*M, &dbgs());

    wcEngine = JIT_to_ExecutionEngine(M);

    wcEngine->finalizeObject();

    delete idb;
//    return wcEngine->getPointerToFunction(main_IR);

    return nullptr;
}

int main(int argc, char *argv[]) {

    cl::ParseCommandLineOptions(argc, argv);

    arrayTest();

    return 0;
}
