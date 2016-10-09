/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/Debug.h>

#include <llvm/Support/CommandLine.h>

#include <toolchain.h>
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/pablo_kernel.h>
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <kernels/pipeline.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/deletion.h>
#include <kernels/stdout_kernel.h>
#include <llvm/IR/TypeBuilder.h>


// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <fcntl.h>
static cl::OptionCategory u8u16Options("u8u16 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(u8u16Options));

static cl::opt<bool> segmentPipelineParallel("enable-segment-pipeline-parallel", cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(u8u16Options));

//
//
//
namespace pablo {

PabloFunction * u8u16_pablo() {
    //  input: 8 basis bit streams
    //  output: 16 u8-indexed streams, + delmask stream + error stream
    PabloFunction * function = PabloFunction::Create("u8u16", 8, 18);
    cc::CC_Compiler ccc(*function);
    
    PabloBuilder pBuilder(ccc.getBuilder().getPabloBlock(), ccc.getBuilder());
    const std::vector<Var *> u8_bits = ccc.getBasisBits();
    // Outputs
    Assign * u16_hi[8];
    Assign * u16_lo[8];
    Assign * delmask;
    Assign * error_mask;
    
    // The logic for processing non-ASCII bytes is to be embedded within an if-hierarchy.
    PabloAST * nonASCII = ccc.compileCC(re::makeCC(0x80, 0xFF));
    
    // Builder for the if statement handling all non-ASCII logic
    PabloBuilder nAb = PabloBuilder::Create(pBuilder);
    // Bits 3 through 7 of a 2-byte prefix are data bits, needed to
    // produce the UTF-16 code unit data ..., 
    PabloAST * bit3a1 = nAb.createAdvance(u8_bits[3], 1);
    PabloAST * bit4a1 = nAb.createAdvance(u8_bits[4], 1);
    PabloAST * bit5a1 = nAb.createAdvance(u8_bits[5], 1);
    PabloAST * bit6a1 = nAb.createAdvance(u8_bits[6], 1);
    PabloAST * bit7a1 = nAb.createAdvance(u8_bits[7], 1);
    
    // Entry condition for 3 or 4 byte sequences: we have a prefix byte in the range 0xE0-0xFF.
    PabloAST * pfx34 = ccc.compileCC(re::makeCC(0xE0, 0xFF), nAb);
    // Builder for the if statement handling all logic for 3- and 4-byte sequences.
    PabloBuilder p34b = PabloBuilder::Create(nAb);
    // Bits 4 through 7 of a 3-byte prefix are data bits.  They must be moved
    // to the final position of the 3-byte sequence.
    PabloAST * bit2a1 = p34b.createAdvance(u8_bits[2], 1);
    PabloAST * bit4a2 = p34b.createAdvance(bit4a1, 1);
    PabloAST * bit5a2 = p34b.createAdvance(bit5a1, 1);
    PabloAST * bit6a2 = p34b.createAdvance(bit6a1, 1);
    PabloAST * bit7a2 = p34b.createAdvance(bit7a1, 1);
    //
    // Logic for 4-byte UTF-8 sequences
    //
    // Entry condition  or 4 byte sequences: we have a prefix byte in the range 0xF0-0xFF.
    PabloAST * pfx4 = ccc.compileCC(re::makeCC(0xF0, 0xFF), p34b);
    // Builder for the if statement handling all logic for 4-byte sequences only.
    PabloBuilder p4b = PabloBuilder::Create(p34b);
    // Illegal 4-byte sequences
    PabloAST * F0 = ccc.compileCC(re::makeCC(0xF0), p4b);
    PabloAST * F4 = ccc.compileCC(re::makeCC(0xF4), p4b);
    PabloAST * F0_err = p4b.createAnd(p4b.createAdvance(F0, 1), ccc.compileCC(re::makeCC(0x80, 0x8F), p4b));
    PabloAST * F4_err = p4b.createAnd(p4b.createAdvance(F4, 1), ccc.compileCC(re::makeCC(0x90, 0xBF), p4b));
    PabloAST * F5_FF = ccc.compileCC(re::makeCC(0xF5, 0xFF), p4b);
    Assign * FX_err = p4b.createAssign("FX_err", p4b.createOr(F5_FF, p4b.createOr(F0_err, F4_err)));
    //
    // 4-byte prefixes have a scope that extends over the next 3 bytes.
    Assign * u8scope42 = p4b.createAssign("u8scope42", p4b.createAdvance(pfx4, 1));
    Assign * u8scope43 = p4b.createAssign("u8scope43", p4b.createAdvance(u8scope42, 1));
    Assign * u8scope44 = p4b.createAssign("u8scope44", p4b.createAdvance(u8scope43, 1));
    //
    
    //  From the 4-byte sequence 11110abc 10defghi 10jklmno 10pqrstu,
    //  we must calculate the value abcde - 1 to produce the bit values
    //  for u16_hi6, hi7, lo0, lo1 at the scope43 position.
    Assign * s43_lo1 = p4b.createAssign("scope43_lo1", p4b.createAnd(u8scope43, p4b.createNot(bit3a1)));           // e - 1
    Assign * s43_lo0 = p4b.createAssign("scope43_lo0", p4b.createAnd(u8scope43, p4b.createXor(bit2a1, s43_lo1)));  // d - borrow
    PabloAST * brw1 = p4b.createAnd(s43_lo1, p4b.createNot(bit2a1));
    Assign * s43_hi7 = p4b.createAssign("scope43_hi7", p4b.createAnd(u8scope43, p4b.createXor(bit7a2, brw1)));     // c - borrow
    PabloAST * brw2 = p4b.createAnd(brw1, p4b.createNot(bit7a2));
    Assign * s43_hi6 = p4b.createAssign("scope43_hi6", p4b.createAnd(u8scope43, p4b.createXor(bit6a2, brw2)));     // b - borrow
    //
    Assign * s43_lo2 = p4b.createAssign("scope43_lo2", p4b.createAnd(u8scope43, bit4a1));
    Assign * s43_lo3 = p4b.createAssign("scope43_lo3", p4b.createAnd(u8scope43, bit5a1));
    Assign * s43_lo4 = p4b.createAssign("scope43_lo4", p4b.createAnd(u8scope43, bit6a1));
    Assign * s43_lo5 = p4b.createAssign("scope43_lo5", p4b.createAnd(u8scope43, bit7a1));
    Assign * s43_lo6 = p4b.createAssign("scope43_lo6", p4b.createAnd(u8scope43, u8_bits[2]));
    Assign * s43_lo7 = p4b.createAssign("scope43_lo7", p4b.createAnd(u8scope43, u8_bits[3]));
    //
    //
    p34b.createIf(pfx4,
                  {FX_err, u8scope42, u8scope43, u8scope44, s43_hi6, s43_hi7,
                   s43_lo0, s43_lo1, s43_lo2, s43_lo3, s43_lo4, s43_lo5, s43_lo6, s43_lo7},
                   p4b);
    //
    // Combined logic for 3 and 4 byte sequences
    //
    PabloAST * pfx3 = ccc.compileCC(re::makeCC(0xE0, 0xEF), p34b);
    Assign * u8scope32 = p34b.createAssign("u8scope32", p34b.createAdvance(pfx3, 1));
    Assign * u8scope33 = p34b.createAssign("u8scope33", p34b.createAdvance(u8scope32, 1));

    // Illegal 3-byte sequences
    PabloAST * E0 = ccc.compileCC(re::makeCC(0xE0), p34b);
    PabloAST * ED = ccc.compileCC(re::makeCC(0xED), p34b);
    PabloAST * E0_err = p34b.createAnd(p34b.createAdvance(E0, 1), ccc.compileCC(re::makeCC(0x80, 0x9F), p34b));
    PabloAST * ED_err = p34b.createAnd(p34b.createAdvance(ED, 1), ccc.compileCC(re::makeCC(0xA0, 0xBF), p34b));
    Assign * EX_FX_err = p34b.createAssign("EX_FX_err", p34b.createOr(p34b.createOr(E0_err, ED_err), FX_err));
    // Two surrogate UTF-16 units are computed at the 3rd and 4th positions of 4-byte sequences.
    PabloAST * surrogate = p34b.createOr(u8scope43, u8scope44);
    
    Assign * p34del = p34b.createAssign("p34del", p34b.createOr(u8scope32, u8scope42));


    // The high 5 bits of the UTF-16 code unit are only nonzero for 3 and 4-byte
    // UTF-8 sequences.
    u16_hi[0] = p34b.createAssign("u16_hi0", p34b.createOr(p34b.createAnd(u8scope33, bit4a2), surrogate));
    u16_hi[1] = p34b.createAssign("u16_hi1", p34b.createOr(p34b.createAnd(u8scope33, bit5a2), surrogate));
    u16_hi[2] = p34b.createAssign("u16_hi2", p34b.createAnd(u8scope33, bit6a2));
    u16_hi[3] = p34b.createAssign("u16_hi3", p34b.createOr(p34b.createAnd(u8scope33, bit7a2), surrogate));
    u16_hi[4] = p34b.createAssign("u16_hi4", p34b.createOr(p34b.createAnd(u8scope33, bit2a1), surrogate));
    
    //
    nAb.createIf(pfx34, 
                 {u8scope33, EX_FX_err, p34del, 
                  u16_hi[0], u16_hi[1], u16_hi[2], u16_hi[3], u16_hi[4], u8scope44, s43_hi6, s43_hi7,
                  s43_lo0, s43_lo1, s43_lo2, s43_lo3, s43_lo4, s43_lo5, s43_lo6, s43_lo7},
                 p34b);
    //
    // Combined logic for 2, 3 and 4 byte sequences
    //
    PabloAST * pfx2 = ccc.compileCC(re::makeCC(0xC0, 0xDF), nAb);
    PabloAST * u8scope22 = nAb.createAdvance(pfx2, 1);
    Assign * u8lastscope = nAb.createAssign("u8lastscope", nAb.createOr(u8scope22, nAb.createOr(u8scope33, u8scope44)));
    PabloAST * u8anyscope = nAb.createOr(u8lastscope, p34del);

    PabloAST * C0_C1_err = ccc.compileCC(re::makeCC(0xC0, 0xC1), nAb);
    PabloAST * scope_suffix_mismatch = nAb.createXor(u8anyscope, ccc.compileCC(re::makeCC(0x80, 0xBF), nAb));
    error_mask = nAb.createAssign("errormask", nAb.createOr(scope_suffix_mismatch, nAb.createOr(C0_C1_err, EX_FX_err)));
    delmask = nAb.createAssign("delmask", nAb.createOr(p34del, ccc.compileCC(re::makeCC(0xC0, 0xFF), nAb)));
    
    // The low 3 bits of the high byte of the UTF-16 code unit as well as the high bit of the
    // low byte are only nonzero for 2, 3 and 4 byte sequences.
    u16_hi[5] = nAb.createAssign("u16_hi5", nAb.createOr(nAb.createAnd(u8lastscope, bit3a1), u8scope44));
    u16_hi[6] = nAb.createAssign("u16_hi6", nAb.createOr(nAb.createAnd(u8lastscope, bit4a1), s43_hi6));
    u16_hi[7] = nAb.createAssign("u16_hi7", nAb.createOr(nAb.createAnd(u8lastscope, bit5a1), s43_hi7));
    u16_lo[0] = nAb.createAssign("u16_lo0", nAb.createOr(nAb.createAnd(u8lastscope, bit6a1), s43_lo0));
    Assign * p234_lo1 = nAb.createAssign("p234_lo1", nAb.createOr(nAb.createAnd(u8lastscope, bit7a1), s43_lo1));

    pBuilder.createIf(nonASCII, 
                      {error_mask, delmask, u8lastscope,
                       u16_hi[0], u16_hi[1], u16_hi[2], u16_hi[3], u16_hi[4], u16_hi[5], u16_hi[6], u16_hi[7],
                       u16_lo[0], p234_lo1, s43_lo2, s43_lo3, s43_lo4, s43_lo5, s43_lo6, s43_lo7},
                      nAb);
    //
    //
    PabloAST * ASCII = ccc.compileCC(re::makeCC(0x0, 0x7F));
    PabloAST * last_byte = pBuilder.createOr(ASCII, u8lastscope);
    u16_lo[1] = pBuilder.createAssign("u16_lo1", pBuilder.createOr(pBuilder.createAnd(ASCII, u8_bits[1]), p234_lo1));
    u16_lo[2] = pBuilder.createAssign("u16_lo2", pBuilder.createOr(pBuilder.createAnd(last_byte, u8_bits[2]), s43_lo2));
    u16_lo[3] = pBuilder.createAssign("u16_lo3", pBuilder.createOr(pBuilder.createAnd(last_byte, u8_bits[3]), s43_lo3));
    u16_lo[4] = pBuilder.createAssign("u16_lo4", pBuilder.createOr(pBuilder.createAnd(last_byte, u8_bits[4]), s43_lo4));
    u16_lo[5] = pBuilder.createAssign("u16_lo5", pBuilder.createOr(pBuilder.createAnd(last_byte, u8_bits[5]), s43_lo5));
    u16_lo[6] = pBuilder.createAssign("u16_lo6", pBuilder.createOr(pBuilder.createAnd(last_byte, u8_bits[6]), s43_lo6));
    u16_lo[7] = pBuilder.createAssign("u16_lo7", pBuilder.createOr(pBuilder.createAnd(last_byte, u8_bits[7]), s43_lo7));
    
    for (unsigned i = 0; i < 8; i++) {
        function->setResult(i, pBuilder.createAssign("u16_hi" + std::to_string(i), u16_hi[i]));
        function->setResult(i+8, pBuilder.createAssign("u16_lo" + std::to_string(i), u16_lo[i]));
    }
    function->setResult(16, pBuilder.createAssign("delbits", delmask));
    function->setResult(17, pBuilder.createAssign("errors", error_mask));

    return function;
}
}



using namespace kernel;
using namespace parabix;

const unsigned u16OutputBlocks = 64;

Function * u8u16Pipeline(Module * mMod, IDISA::IDISA_Builder * iBuilder, pablo::PabloFunction * function) {
    Type * mBitBlockType = iBuilder->getBitBlockType();

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;
    
    ExternalFileBuffer ByteStream(iBuilder, StreamSetType(1, i8));
    //SingleBlockBuffer BasisBits(iBuilder, StreamSetType(8, i1));
    CircularBuffer BasisBits(iBuilder, StreamSetType(8, i1), segmentSize * bufferSegments);

    //SingleBlockBuffer U8u16Bits(iBuilder, StreamSetType(18, i1));
    CircularBuffer U8u16Bits(iBuilder, StreamSetType(18, i1), segmentSize * bufferSegments);

    //SingleBlockBuffer U16Bits(iBuilder, StreamSetType(16, i1));
    CircularBuffer U16Bits(iBuilder, StreamSetType(16, i1), segmentSize * bufferSegments);
    
    //SingleBlockBuffer DeletionCounts(iBuilder, StreamSetType(1, i1));
    CircularBuffer DeletionCounts(iBuilder, StreamSetType(1, i1), segmentSize * bufferSegments );
    
    LinearBuffer U16out(iBuilder, StreamSetType(1, i16), segmentSize * bufferSegments + 2);

    s2pKernel  s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});

    pablo_function_passes(function);
    pablo::PabloKernel  u8u16k(iBuilder, "u8u16", function, {});
    u8u16k.generateKernel({&BasisBits}, {&U8u16Bits});
    
    deletionKernel delK(iBuilder, iBuilder->getBitBlockWidth()/16, 16);
    delK.generateKernel({&U8u16Bits}, {&U16Bits, &DeletionCounts});
    
    p2s_16Kernel_withCompressedOutput p2sk(iBuilder);
    p2sk.generateKernel({&U16Bits, &DeletionCounts}, {&U16out});
    
    stdOutKernel stdoutK(iBuilder, 16);
    stdoutK.generateKernel({&U16out}, {});

    
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(mMod->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const voidPtrTy = TypeBuilder<void *, false>::get(mMod->getContext());

    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", voidTy, inputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));
        

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    U8u16Bits.allocateBuffer();
    U16Bits.allocateBuffer();
    DeletionCounts.allocateBuffer();
    U16out.allocateBuffer();

    Value * s2pInstance = s2pk.createInstance({});
    Value * u8u16Instance = u8u16k.createInstance({});
    Value * delInstance = delK.createInstance({});
    Value * p2sInstance = p2sk.createInstance({});
    Value * stdoutInstance = stdoutK.createInstance({});
    
    Type * pthreadTy = size_ty;
    FunctionType * funVoidPtrVoidTy = FunctionType::get(voidTy, int8PtrTy, false);
    
    Function * pthreadCreateFunc = cast<Function>(mMod->getOrInsertFunction("pthread_create",
                                                                         int32ty,
                                                                         pthreadTy->getPointerTo(),
                                                                         voidPtrTy,
                                                                         static_cast<Type *>(funVoidPtrVoidTy)->getPointerTo(),
                                                                         voidPtrTy, nullptr));
    pthreadCreateFunc->setCallingConv(llvm::CallingConv::C);
    Function * pthreadJoinFunc = cast<Function>(mMod->getOrInsertFunction("pthread_join",
                                                                       int32ty,
                                                                       pthreadTy,
                                                                       PointerType::get(int8PtrTy, 0), nullptr));
    pthreadJoinFunc->setCallingConv(llvm::CallingConv::C);
    
    Function * pthreadExitFunc = cast<Function>(mMod->getOrInsertFunction("pthread_exit",
                                                                       voidTy, 
                                                                       voidPtrTy, nullptr));
    pthreadExitFunc->addFnAttr(llvm::Attribute::NoReturn);
    pthreadExitFunc->setCallingConv(llvm::CallingConv::C);

    if (segmentPipelineParallel){
        generateSegmentParallelPipeline(iBuilder, {&s2pk, &u8u16k, &delK, &p2sk, &stdoutK}, {s2pInstance, u8u16Instance, delInstance, p2sInstance, stdoutInstance}, fileSize);
    }
    else{
        generatePipelineLoop(iBuilder, {&s2pk, &u8u16k, &delK, &p2sk, &stdoutK}, {s2pInstance, u8u16Instance, delInstance, p2sInstance, stdoutInstance}, fileSize);
    }

    iBuilder->CreateRetVoid();
    return main;
}





typedef void (*u8u16FunctionType)(char * byte_data, size_t filesize);

static ExecutionEngine * u8u16Engine = nullptr;

u8u16FunctionType u8u16CodeGen(void) {
    LLVMContext TheContext;                            
    Module * M = new Module("u8u16", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    pablo::PabloFunction * function = pablo::u8u16_pablo();
    
    llvm::Function * main_IR = u8u16Pipeline(M, idb, function);
    
    verifyModule(*M, &dbgs());
    //std::cerr << "ExecuteKernels(); done\n";
    u8u16Engine = JIT_to_ExecutionEngine(M);
    
    u8u16Engine->finalizeObject();
    //std::cerr << "finalizeObject(); done\n";

    delete idb;
    return reinterpret_cast<u8u16FunctionType>(u8u16Engine->getPointerToFunction(main_IR));
}

void u8u16(u8u16FunctionType fn_ptr, const std::string & fileName) {
    std::string mFileName = fileName;
    size_t mFileSize;
    char * mFileBuffer;
    
    const boost::filesystem::path file(mFileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return;
    }
    
    mFileSize = file_size(file);
    boost::iostreams::mapped_file_source mFile;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        try {
            mFile.open(mFileName);
        } catch (std::exception &e) {
            std::cerr << "Error: Boost mmap of " << mFileName << ": " << e.what() << std::endl;
            return;
        }
        mFileBuffer = const_cast<char *>(mFile.data());
    }
    //std::cerr << "mFileSize =" << mFileSize << "\n";
    //std::cerr << "fn_ptr =" << std::hex << reinterpret_cast<intptr_t>(fn_ptr) << "\n";

    fn_ptr(mFileBuffer, mFileSize);

    mFile.close();
    
}


int main(int argc, char *argv[]) {
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&u8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);

    u8u16FunctionType fn_ptr = u8u16CodeGen();

    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        u8u16(fn_ptr, inputFiles[i]);
    }

    return 0;
}

                       
