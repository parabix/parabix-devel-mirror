/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/core/idisa_target.h>                   // for GetIDISA_Builder
#include <re/cc/cc_compiler.h>                        // for CC_Compiler
#include <re/alphabet/alphabet.h>
#include <re/cc/utf16gen.h>
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

static cl::OptionCategory u32u16Options("u32u16 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(u32u16Options));
static cl::opt<std::string> OutputEncoding("encoding", cl::desc("Output encoding (default: UTF-16)"), cl::init("UTF-16"), cl::cat(u32u16Options));

typedef void (*u32u16FunctionType)(uint32_t fd);

u32u16FunctionType u32u16_gen (CPUDriver & driver, cc::ByteNumbering byteNumbering) {
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
    
    // Buffer for supplementary plane basis bits.
    StreamSet * const u16_SMP_basis = P->CreateStreamSet(4);
    P->CreateKernelCall<UTF16_SupplementaryBasis>(u32basis, u16_SMP_basis);

    // Buffers for calculated deposit masks.
    StreamSet * const u16fieldMask = P->CreateStreamSet();
    StreamSet * const u16final = P->CreateStreamSet();
    StreamSet * const u16initial = P->CreateStreamSet();

    // Intermediate buffers for deposited bits
    StreamSet * const SMP4_0 = P->CreateStreamSet(4);
    StreamSet * const deposit15_10 = P->CreateStreamSet(6);
    StreamSet * const deposit9_0 = P->CreateStreamSet(10);

    // Final buffers for computed UTF-16 basis bits and code unit stream.
    StreamSet * const u16basis = P->CreateStreamSet(16);
    StreamSet * const u16bytes = P->CreateStreamSet(1, 16);

    // Calculate the u16final deposit mask.
    StreamSet * const extractionMask = P->CreateStreamSet();
    P->CreateKernelCall<UTF16fieldDepositMask>(u32basis, u16fieldMask, extractionMask);
    P->CreateKernelCall<StreamCompressKernel>(u16fieldMask, extractionMask, u16final);
    P->CreateKernelCall<UTF16_InitialMask>(u16final, u16initial);

    SpreadByMask(P, u16initial, u16_SMP_basis, SMP4_0);
    SpreadByMask(P, u16initial, u32basis, deposit15_10, /* inputOffset = */ 10);
    SpreadByMask(P, u16final, u32basis, deposit9_0);

    P->CreateKernelCall<UTF16assembly>(SMP4_0, deposit15_10, deposit9_0, u16final,
                                      u16basis);
    P->CreateKernelCall<P2S16Kernel>(u16basis, u16bytes, byteNumbering);
    P->CreateKernelCall<StdOutKernel>(u16bytes);
    return reinterpret_cast<u32u16FunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&u32u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    cc::ByteNumbering byteNumbering = cc::ByteNumbering::BigEndian;

    if ((OutputEncoding == "UTF16LE") || (OutputEncoding == "UTF-16LE")) {
        byteNumbering = cc::ByteNumbering::LittleEndian;
    } else if ((OutputEncoding == "UTF16BE") || (OutputEncoding == "UTF-16BE")) {
        byteNumbering = cc::ByteNumbering::BigEndian;
    } else if ((OutputEncoding != "UTF16") &&  (OutputEncoding != "UTF-16")) {
        llvm::report_fatal_error("Unsupported encoding.");
    }
    CPUDriver pxDriver("u32u16");
    auto u32u16Function = u32u16_gen(pxDriver, byteNumbering);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        u32u16Function(fd);
        close(fd);
    }
    return 0;
}
