/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/core/idisa_target.h>                   // for GetIDISA_Builder
#include <re/cc/cc_compiler.h>                        // for CC_Compiler
#include <pablo/bixnum/utf8gen.h>
#include <kernel/streamutils/deletion.h>                      // for DeletionKernel
#include <kernel/streamutils/swizzle.h>
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
#include <toolchain/pablo_toolchain.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_debugprint.h>
#include <toolchain/toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
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
#include <kernel/util/debug_display.h>
#include <kernel/streamutils/streams_merge.h>
#include <kernel/scan/scanmatchgen.h>
#include <re/adt/re_cc.h>
#include <pablo/pe_zeroes.h>        // for Zeroes

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace codegen;
using namespace re;
using BuilderRef = Kernel::BuilderRef;

static cl::OptionCategory u16u8Options("u16u8 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(u16u8Options));

typedef void (*u16u8FunctionType)(uint32_t fd);

u16u8FunctionType u16u8_gen (CPUDriver & driver) {

    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const codeUnitStream = P->CreateStreamSet(1, 16);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, codeUnitStream);

    // Source buffers for transposed UTF-16 basis bits.
    StreamSet * const u16basis = P->CreateStreamSet(16);
    P->CreateKernelCall<S2P_16Kernel>(codeUnitStream, u16basis, cc::ByteNumbering::LittleEndian);
    //P->CreateKernelCall<DebugDisplayKernel>("u16basis", u16basis);
    
    StreamSet * const prefix = P->CreateStreamSet();
    StreamSet * const suffix = P->CreateStreamSet();
    StreamSet * const len4 = P->CreateStreamSet();
    StreamSet * selectors = P->CreateStreamSet();

    P->CreateKernelCall<U16U8index>(u16basis, prefix, suffix, len4, selectors);

    StreamSet * u16Bits = P->CreateStreamSet(21);
    StreamSet * u32basis = P->CreateStreamSet(21);
    P->CreateKernelCall<shuffle>(u16basis, u16Bits, prefix, suffix, len4);
    P->CreateKernelCall<FieldCompressKernel>(Select(selectors, {0}),
                                            SelectOperationList{Select(u16Bits, streamutils::Range(0, 21))},
                                            u32basis);
    //P->CreateKernelCall<DebugDisplayKernel>("u32basis", u32basis);

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
    P->CreateKernelCall<StreamCompressKernel>(extractionMask, u8fieldMask, u8final);

    P->CreateKernelCall<UTF8_DepositMasks>(u8final, u8initial, u8mask12_17, u8mask6_11);

    SpreadByMask(P, u8initial, u32basis, deposit18_20, /* inputOffset = */ 18);
    SpreadByMask(P, u8mask12_17, u32basis, deposit12_17, /* inputOffset = */ 12);
    SpreadByMask(P, u8mask6_11, u32basis, deposit6_11, /* inputOffset = */ 6);
    SpreadByMask(P, u8final, u32basis, deposit0_5, /* inputOffset = */ 0);

    P->CreateKernelCall<UTF8assembly>(deposit18_20, deposit12_17, deposit6_11, deposit0_5,
                                      u8initial, u8final, u8mask6_11, u8mask12_17,
                                      u8basis);
    //P->CreateKernelCall<DebugDisplayKernel>("u8basis", u8basis);
    P->CreateKernelCall<P2SKernel>(u8basis, u8bytes);

    P->CreateKernelCall<StdOutKernel>(u8bytes);

    return reinterpret_cast<u16u8FunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&u16u8Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver pxDriver("u16u8");
    auto u16u8Function = u16u8_gen(pxDriver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        u16u8Function(fd);
        close(fd);
    }
    return 0;
}
