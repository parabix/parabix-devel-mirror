/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/core/idisa_target.h>                   // for GetIDISA_Builder
#include <re/cc/cc_compiler.h>                        // for CC_Compiler
#include <re/cc/utf8gen.h>
#include <kernel/streamutils/deletion.h>                      // for DeletionKernel
#include <kernel/io/source_kernel.h>
#include <kernel/basis/p2s_kernel.h>                    // for P2S16KernelWithCom...
#include <kernel/basis/s2p_kernel.h>                    // for S2PKernel
#include <kernel/io/stdout_kernel.h>                 // for StdOutKernel_
#include <kernel/streamutils/pdep_kernel.h>
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <pablo/pablo_kernel.h>                    // for PabloKernel
#include <pablo/pablo_toolchain.h>                 // for pablo_function_passes
#include <kernel/core/kernel_builder.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <kernel/core/streamset.h>
#include <kernel/util/hex_convert.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Compiler.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <fcntl.h>
#include <kernel/pipeline/pipeline_builder.h>

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace codegen;

static cl::OptionCategory u32u8Options("u32u8 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(u32u8Options));

typedef void (*u32u8FunctionType)(uint32_t fd);

u32u8FunctionType u32u8_gen (CPUDriver & driver) {

    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const codeUnitStream = P->CreateStreamSet(1, 32);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, codeUnitStream);

    // Source buffers for transposed UTF-32 basis bits.
    StreamSet * const u32basis = P->CreateStreamSet(21);
    P->CreateKernelCall<S2P_21Kernel>(codeUnitStream, u32basis);

    // Buffers for calculated deposit masks.
    StreamSet * const u8fieldMask = P->CreateStreamSet();
    StreamSet * const u8final = P->CreateStreamSet();
    StreamSet * const u8initial = P->CreateStreamSet();
    StreamSet * const u8mask12_17 = P->CreateStreamSet();
    StreamSet * const u8mask6_11 = P->CreateStreamSet();

    // Intermediate buffers for deposited bits
    StreamSet * const deposit18_20 = P->CreateStreamSet(3);
    StreamSet * const deposit12_17 = P->CreateStreamSet(6);
    StreamSet * const deposit6_11 = P->CreateStreamSet(6);
    StreamSet * const deposit0_5 = P->CreateStreamSet(6);

    // Final buffers for computed UTF-8 basis bits and byte stream.
    StreamSet * const u8basis = P->CreateStreamSet(8);
    StreamSet * const u8bytes = P->CreateStreamSet(1, 8);

    // Calculate the u8final deposit mask.
    StreamSet * const extractionMask = P->CreateStreamSet();
    P->CreateKernelCall<UTF8fieldDepositMask>(u32basis, u8fieldMask, extractionMask);
    P->CreateKernelCall<StreamCompressKernel>(u8fieldMask, extractionMask, u8final);

    P->CreateKernelCall<UTF8_DepositMasks>(u8final, u8initial, u8mask12_17, u8mask6_11);

    SpreadByMask(P, u8initial, u32basis, deposit18_20, /* inputOffset = */ 18);
    SpreadByMask(P, u8mask12_17, u32basis, deposit12_17, /* inputOffset = */ 12);
    SpreadByMask(P, u8mask6_11, u32basis, deposit6_11, /* inputOffset = */ 6);
    SpreadByMask(P, u8final, u32basis, deposit0_5, /* inputOffset = */ 0);

    P->CreateKernelCall<UTF8assembly>(deposit18_20, deposit12_17, deposit6_11, deposit0_5,
                                      u8initial, u8final, u8mask6_11, u8mask12_17,
                                      u8basis);

    P->CreateKernelCall<P2SKernel>(u8basis, u8bytes);

    P->CreateKernelCall<StdOutKernel>(u8bytes);

    return reinterpret_cast<u32u8FunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&u32u8Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver pxDriver("u32u8");
    auto u32u8Function = u32u8_gen(pxDriver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        u32u8Function(fd);
        close(fd);
    }
    return 0;
}
