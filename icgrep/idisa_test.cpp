/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <vector>
#include <string>
#include <toolchain/toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <kernels/kernel_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/interface.h>
#include <kernels/streamset.h>
#include <kernels/source_kernel.h>
#include <kernels/hex_convert.h>
#include <kernels/s2p_kernel.h>
#include <kernels/stdout_kernel.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

using namespace llvm;

static cl::OptionCategory testFlags("Command Flags", "test options");

static cl::opt<std::string> TestOperation(cl::Positional, cl::desc("Operation to test"), cl::Required, cl::cat(testFlags));

static cl::opt<int> TestFieldWidth(cl::Positional, cl::desc("Test field width (default 64)."), cl::init(64), cl::Required, cl::cat(testFlags));

static cl::opt<std::string> Operand1TestFile(cl::Positional, cl::desc("Operand 1 data file."), cl::Required, cl::cat(testFlags));
static cl::opt<std::string> Operand2TestFile(cl::Positional, cl::desc("Operand 2 data file."), cl::Required, cl::cat(testFlags));
static cl::opt<std::string> TestOutputFile("o", cl::desc("Test output file."), cl::cat(testFlags));
static cl::opt<bool> QuietMode("q", cl::desc("Suppress output, set the return code only."), cl::cat(testFlags));
static cl::opt<int> ShiftLimit("ShiftLimit", cl::desc("Upper limit for the shift operand (2nd operand) of sllv, srlv, srav."), cl::init(0));
static cl::opt<int> Immediate("i", cl::desc("Immediate value for mvmd_dslli"), cl::init(1));

class ShiftLimitKernel : public kernel::BlockOrientedKernel {
public:
    ShiftLimitKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned fw, unsigned limit);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & kb) override;
private:
    const unsigned mTestFw;
    const unsigned mShiftLimit;
};

ShiftLimitKernel::ShiftLimitKernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned fw, unsigned limit)
: kernel::BlockOrientedKernel("shiftLimit" + std::to_string(fw) + "_" + std::to_string(limit),
                              {kernel::Binding{b->getStreamSetTy(1, fw), "shiftOperand"}},
                              {kernel::Binding{b->getStreamSetTy(1, fw), "limitedShift"}},
                              {}, {}, {}),
mTestFw(fw), mShiftLimit(limit) {}

void ShiftLimitKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & kb) {
    Type * fwTy = kb->getIntNTy(mTestFw);
    Constant * const ZeroConst = kb->getSize(0);
    Value * shiftOperand = kb->loadInputStreamBlock("shiftOperand", ZeroConst);
    unsigned fieldCount = kb->getBitBlockWidth()/mTestFw;
    Value * limited = kb->simd_umin(mTestFw, shiftOperand, ConstantVector::getSplat(fieldCount, ConstantInt::get(fwTy, mShiftLimit)));
    kb->storeOutputStreamBlock("limitedShift", ZeroConst, limited);
}

class IdisaBinaryOpTestKernel : public kernel::MultiBlockKernel {
public:
    IdisaBinaryOpTestKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm=0);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const std::string mIdisaOperation;
    const unsigned mTestFw;
    const unsigned mImmediateShift;
};

IdisaBinaryOpTestKernel::IdisaBinaryOpTestKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm)
: kernel::MultiBlockKernel(idisa_op + std::to_string(fw) + "_test",
     {kernel::Binding{b->getStreamSetTy(1, 1), "operand1"}, kernel::Binding{b->getStreamSetTy(1, 1), "operand2"}},
     {kernel::Binding{b->getStreamSetTy(1, 1), "result"}},
     {}, {}, {}),
mIdisaOperation(idisa_op), mTestFw(fw), mImmediateShift(imm) {}

void IdisaBinaryOpTestKernel::generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZeroConst = kb->getSize(0);
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZeroConst, entry);
    Value * operand1 = kb->loadInputStreamBlock("operand1", ZeroConst, blockOffsetPhi);
    Value * operand2 = kb->loadInputStreamBlock("operand2", ZeroConst, blockOffsetPhi);
    Value * result = nullptr;
    if (mIdisaOperation == "simd_add") {
        result = kb->simd_add(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_sub") {
        result = kb->simd_sub(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_mult") {
        result = kb->simd_mult(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_eq") {
        result = kb->simd_eq(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_gt") {
        result = kb->simd_gt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ugt") {
        result = kb->simd_ugt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_uge") {
        result = kb->simd_uge(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_lt") {
        result = kb->simd_lt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ult") {
        result = kb->simd_ult(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ule") {
        result = kb->simd_ule(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_max") {
        result = kb->simd_max(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_min") {
        result = kb->simd_min(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_umax") {
        result = kb->simd_umax(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_umin") {
        result = kb->simd_umin(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_sllv") {
        result = kb->simd_sllv(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_srlv") {
        result = kb->simd_srlv(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_pext") {
        result = kb->simd_pext(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_pdep") {
        result = kb->simd_pdep(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "hsimd_packh") {
        result = kb->hsimd_packh(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "hsimd_packl") {
        result = kb->hsimd_packl(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "esimd_mergeh") {
        result = kb->esimd_mergeh(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "esimd_mergel") {
        result = kb->esimd_mergel(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "mvmd_shuffle") {
        result = kb->mvmd_shuffle(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "mvmd_compress") {
        result = kb->mvmd_compress(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "mvmd_dslli") {
        result = kb->mvmd_dslli(mTestFw, operand1, operand2, mImmediateShift);
    } else {
        llvm::report_fatal_error("Binary operation " + mIdisaOperation + " is unknown to the IdisaBinaryOpTestKernel kernel.");
    }
    kb->storeOutputStreamBlock("result", ZeroConst, blockOffsetPhi, kb->bitCast(result));
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

class IdisaBinaryOpCheckKernel : public kernel::BlockOrientedKernel {
public:
    IdisaBinaryOpCheckKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm=0);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & kb) override;
private:
    const std::string mIdisaOperation;
    const unsigned mTestFw;
    const unsigned mImmediateShift;
};

IdisaBinaryOpCheckKernel::IdisaBinaryOpCheckKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm)
: kernel::BlockOrientedKernel(idisa_op + std::to_string(fw) + "_check" + std::to_string(QuietMode),
                           {kernel::Binding{b->getStreamSetTy(1, 1), "operand1"},
                            kernel::Binding{b->getStreamSetTy(1, 1), "operand2"},
                            kernel::Binding{b->getStreamSetTy(1, 1), "test_result"}},
                           {kernel::Binding{b->getStreamSetTy(1, 1), "expected_result"}},
                           {}, {kernel::Binding{b->getSizeTy(), "totalFailures"}}, {}),
mIdisaOperation(idisa_op), mTestFw(fw), mImmediateShift(imm) {}

void IdisaBinaryOpCheckKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & kb) {
    Type * fwTy = kb->getIntNTy(mTestFw);
    BasicBlock * reportFailure = kb->CreateBasicBlock("reportFailure");
    BasicBlock * continueTest = kb->CreateBasicBlock("continueTest");
    Constant * const ZeroConst = kb->getSize(0);
    Value * operand1Block = kb->loadInputStreamBlock("operand1", ZeroConst);
    Value * operand2Block = kb->loadInputStreamBlock("operand2", ZeroConst);
    Value * resultBlock = kb->loadInputStreamBlock("test_result", ZeroConst);
    unsigned fieldCount = kb->getBitBlockWidth()/mTestFw;
    Value * expectedBlock = kb->allZeroes();
    if (mIdisaOperation == "mvmd_shuffle") {
        for (unsigned i = 0; i < fieldCount; i++) {
            Value * idx = kb->CreateURem(kb->mvmd_extract(mTestFw, operand2Block, i), ConstantInt::get(fwTy, fieldCount));
            Value * elt = kb->CreateExtractElement(kb->fwCast(mTestFw, operand1Block), kb->CreateZExtOrTrunc(idx, kb->getInt32Ty()));
            expectedBlock = kb->mvmd_insert(mTestFw, expectedBlock, elt, i);
        }
    } else if (mIdisaOperation == "mvmd_dslli") {
        for (unsigned i = 0; i < fieldCount; i++) {
            Value * elt = nullptr;
            if (i < mImmediateShift) elt = kb->mvmd_extract(mTestFw, operand2Block, fieldCount - mImmediateShift + i);
            else elt = kb->mvmd_extract(mTestFw, operand1Block, i - mImmediateShift);
            expectedBlock = kb->mvmd_insert(mTestFw, expectedBlock, elt, i);
        }
    } else {
        for (unsigned i = 0; i < fieldCount; i++) {
            Value * operand1 = kb->mvmd_extract(mTestFw, operand1Block, i);
            Value * operand2 = kb->mvmd_extract(mTestFw, operand2Block, i);
            Value * expected = nullptr;
            if (mIdisaOperation.substr(0,5) == "simd_") {
                if (mIdisaOperation == "simd_add") {
                    expected = kb->CreateAdd(operand1, operand2);
                } else if (mIdisaOperation == "simd_sub") {
                    expected = kb->CreateSub(operand1, operand2);
                } else if (mIdisaOperation == "simd_mult") {
                    expected = kb->CreateMul(operand1, operand2);
                } else if (mIdisaOperation == "simd_eq") {
                    expected = kb->CreateSExt(kb->CreateICmpEQ(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_gt") {
                    expected = kb->CreateSExt(kb->CreateICmpSGT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_ugt") {
                    expected = kb->CreateSExt(kb->CreateICmpUGT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_uge") {
                    expected = kb->CreateSExt(kb->CreateICmpUGE(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_lt") {
                    expected = kb->CreateSExt(kb->CreateICmpSLT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_ult") {
                    expected = kb->CreateSExt(kb->CreateICmpULT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_ule") {
                    expected = kb->CreateSExt(kb->CreateICmpULE(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_max") {
                    expected = kb->CreateSelect(kb->CreateICmpSGT(operand1, operand2), operand1, operand2);
                } else if (mIdisaOperation == "simd_min") {
                    expected = kb->CreateSelect(kb->CreateICmpSLT(operand1, operand2), operand1, operand2);
                } else if (mIdisaOperation == "simd_umax") {
                    expected = kb->CreateSelect(kb->CreateICmpUGT(operand1, operand2), operand1, operand2);
                } else if (mIdisaOperation == "simd_umin") {
                    expected = kb->CreateSelect(kb->CreateICmpULT(operand1, operand2), operand1, operand2);
                } else if (mIdisaOperation == "simd_sllv") {
                    expected = kb->CreateShl(operand1, operand2);
                } else if (mIdisaOperation == "simd_srlv") {
                    expected = kb->CreateLShr(operand1, operand2);
                } else if (mIdisaOperation == "simd_pext") {
                    Constant * zeroConst = ConstantInt::getNullValue(fwTy);
                    Constant * oneConst = ConstantInt::get(fwTy, 1);
                    expected = zeroConst;
                    Value * out_bit = oneConst;
                    for (unsigned i = 0; i < mTestFw; i++) {
                        Value * i_bit = Constant::getIntegerValue(fwTy, APInt::getOneBitSet(mTestFw, i));
                        Value * operand_i_isSet = kb->CreateICmpEQ(kb->CreateAnd(operand1, i_bit), i_bit);
                        Value * mask_i_isSet = kb->CreateICmpEQ(kb->CreateAnd(operand2, i_bit), i_bit);
                        expected = kb->CreateSelect(kb->CreateAnd(operand_i_isSet, mask_i_isSet), kb->CreateOr(expected, out_bit), expected);
                        out_bit = kb->CreateSelect(mask_i_isSet, kb->CreateAdd(out_bit, out_bit), out_bit);
                    }
                } else if (mIdisaOperation == "simd_pdep") {
                    Constant * zeroConst = ConstantInt::getNullValue(fwTy);
                    Constant * oneConst = ConstantInt::get(fwTy, 1);
                    expected = zeroConst;
                    Value * shft = zeroConst;
                    Value * select_bit = oneConst;
                    for (unsigned i = 0; i < mTestFw; i++) {
                        expected = kb->CreateOr(kb->CreateAnd(operand2, kb->CreateShl(kb->CreateAnd(operand1, select_bit), shft)), expected);
                        Value * i_bit = Constant::getIntegerValue(fwTy, APInt::getOneBitSet(mTestFw, i));
                        Value * mask_i_isSet = kb->CreateICmpEQ(kb->CreateAnd(operand2, i_bit), i_bit);
                        select_bit = kb->CreateSelect(mask_i_isSet, kb->CreateAdd(select_bit, select_bit), select_bit);
                        shft = kb->CreateSelect(mask_i_isSet, shft, kb->CreateAdd(shft, oneConst));
                    }
                } else {
                    llvm::report_fatal_error("Unknown SIMD vertical operation: " + mIdisaOperation);
                }
                expectedBlock = kb->bitCast(kb->mvmd_insert(mTestFw, expectedBlock, expected, i));
            } else if (mIdisaOperation == "hsimd_packh") {
                operand1 = kb->CreateTrunc(kb->CreateLShr(operand1, mTestFw/2), kb->getIntNTy(mTestFw/2));
                operand2 = kb->CreateTrunc(kb->CreateLShr(operand2, mTestFw/2), kb->getIntNTy(mTestFw/2));
                expectedBlock = kb->mvmd_insert(mTestFw/2, expectedBlock, operand1, i);
                expectedBlock = kb->bitCast(kb->mvmd_insert(mTestFw/2, expectedBlock, operand2, fieldCount + i));
            } else if (mIdisaOperation == "hsimd_packl") {
                operand1 = kb->CreateTrunc(operand1, kb->getIntNTy(mTestFw/2));
                operand2 = kb->CreateTrunc(operand2, kb->getIntNTy(mTestFw/2));
                expectedBlock = kb->mvmd_insert(mTestFw/2, expectedBlock, operand1, i);
                expectedBlock = kb->bitCast(kb->mvmd_insert(mTestFw/2, expectedBlock, operand2, fieldCount + i));
            } else if (mIdisaOperation == "esimd_mergeh") {
                if (i >= fieldCount/2) {
                    expectedBlock = kb->mvmd_insert(mTestFw, expectedBlock, operand1, 2*(i - fieldCount/2));
                    expectedBlock = kb->bitCast(kb->mvmd_insert(mTestFw, expectedBlock, operand2, 2*(i - fieldCount/2) + 1));
                }
            } else if (mIdisaOperation == "esimd_mergel") {
                if (i < fieldCount/2) {
                    expectedBlock = kb->mvmd_insert(mTestFw, expectedBlock, operand1, 2*i);
                    expectedBlock = kb->bitCast(kb->mvmd_insert(mTestFw, expectedBlock, operand2, 2*i + 1));
                }
            }
        }
    }
    kb->storeOutputStreamBlock("expected_result", ZeroConst, expectedBlock);
    Value * failures = kb->simd_ugt(mTestFw, kb->CreateXor(resultBlock, expectedBlock), kb->allZeroes());
    Value * anyFailure = kb->bitblock_any(failures);
    Value * failure_count = kb->CreateUDiv(kb->bitblock_popcount(failures), kb->getSize(mTestFw));
    kb->setScalarField("totalFailures", kb->CreateAdd(kb->getScalarField("totalFailures"), failure_count));
    if (!QuietMode) {
        kb->CreateCondBr(anyFailure, reportFailure, continueTest);
        kb->SetInsertPoint(reportFailure);
        kb->CallPrintRegister("operand1", kb->bitCast(operand1Block));
        kb->CallPrintRegister("operand2", kb->bitCast(operand2Block));
        kb->CallPrintRegister(mIdisaOperation + "(" + std::to_string(mTestFw) + ", operand1, operand2)", resultBlock);
        kb->CallPrintRegister("expecting", expectedBlock);
        kb->CreateBr(continueTest);
        kb->SetInsertPoint(continueTest);
    }
}

// Open a file and return its file desciptor.
int32_t openFile(const std::string & fileName, llvm::raw_ostream & msgstrm) {
    if (fileName == "-") {
        return STDIN_FILENO;
    }
    else {
        struct stat sb;
        int32_t fileDescriptor = open(fileName.c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fileDescriptor == -1)) {
            if (errno == EACCES) {
                msgstrm << "idisa_test: " << fileName << ": Permission denied.\n";
            }
            else if (errno == ENOENT) {
                msgstrm << "idisa_test: " << fileName << ": No such file.\n";
            }
            else {
                msgstrm << "idisa_test: " << fileName << ": Failed.\n";
            }
            return fileDescriptor;
        }
        if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
            msgstrm << "idisa_test: " << fileName << ": Is a directory.\n";
            close(fileDescriptor);
            return -1;
        }
        return fileDescriptor;
    }
}

typedef size_t (*IDISAtestFunctionType)(int32_t fd1, int32_t fd2);

using namespace parabix;

void pipelineGen(ParabixDriver & pxDriver) {

    auto & idb = pxDriver.getBuilder();
    Module * m = idb->getModule();
    Value * useMMap = idb->CreateZExt(idb->getTrue(), idb->getInt8Ty());
    const auto bufferSize = codegen::SegmentSize * codegen::BufferSegments;
    
    Type * const int32Ty = idb->getInt32Ty();
    Type * const sizeTy = idb->getSizeTy();

    FunctionType * const mainType = FunctionType::get(sizeTy, {int32Ty, int32Ty}, false);
    Function * const main = cast<Function>(m->getOrInsertFunction("Main", mainType));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();    
    Value * const fileDecriptor1 = &*(args++);
    fileDecriptor1->setName("operand1FileDecriptor");
    Value * const fileDecriptor2 = &*(args++);
    fileDecriptor2->setName("operand2FileDecriptor");

    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    StreamSetBuffer * Operand1HexStream = pxDriver.addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK1 = pxDriver.addKernelInstance<kernel::FDSourceKernel>(idb);
    sourceK1->setInitialArguments({useMMap, fileDecriptor1});
    pxDriver.makeKernelCall(sourceK1, {}, {Operand1HexStream});
    
    StreamSetBuffer * Operand1BitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * hexbinK = pxDriver.addKernelInstance<kernel::HexToBinary>(idb);
    pxDriver.makeKernelCall(hexbinK, {Operand1HexStream}, {Operand1BitStream});
    
    StreamSetBuffer * Operand2HexStream = pxDriver.addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK2 = pxDriver.addKernelInstance<kernel::FDSourceKernel>(idb);
    sourceK2->setInitialArguments({useMMap, fileDecriptor2});
    pxDriver.makeKernelCall(sourceK2, {}, {Operand2HexStream});
    
    StreamSetBuffer * Operand2BitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * hexbinK2 = pxDriver.addKernelInstance<kernel::HexToBinary>(idb);
    pxDriver.makeKernelCall(hexbinK2, {Operand2HexStream}, {Operand2BitStream});
    
    if (ShiftLimit > 0) {
        StreamSetBuffer * PreLimitBitStream = Operand2BitStream;
        Operand2BitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
        kernel::Kernel * limitK = pxDriver.addKernelInstance<ShiftLimitKernel>(idb, TestFieldWidth, ShiftLimit);
        pxDriver.makeKernelCall(limitK, {PreLimitBitStream}, {Operand2BitStream});
    }

    StreamSetBuffer * ResultBitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * testK = pxDriver.addKernelInstance<IdisaBinaryOpTestKernel>(idb, TestOperation, TestFieldWidth, Immediate);
    pxDriver.makeKernelCall(testK, {Operand1BitStream, Operand2BitStream}, {ResultBitStream});
    
    StreamSetBuffer * ExpectedResultBitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * checkK = pxDriver.addKernelInstance<IdisaBinaryOpCheckKernel>(idb, TestOperation, TestFieldWidth, Immediate);
    pxDriver.makeKernelCall(checkK, {Operand1BitStream, Operand2BitStream, ResultBitStream}, {ExpectedResultBitStream});
    
    if (!TestOutputFile.empty()) {
        StreamSetBuffer * ResultHexStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 8), bufferSize);
        kernel::Kernel * binhexK = pxDriver.addKernelInstance<kernel::BinaryToHex>(idb);
        pxDriver.makeKernelCall(binhexK, {ResultBitStream}, {ResultHexStream});
        kernel::Kernel * outK = pxDriver.addKernelInstance<kernel::FileSink>(idb, 8);
        Value * fName = idb->CreatePointerCast(idb->GetString(TestOutputFile.c_str()), idb->getInt8PtrTy());
        outK->setInitialArguments({fName});
        pxDriver.makeKernelCall(outK, {ResultHexStream}, {});
   }
    
    pxDriver.generatePipelineIR();
    idb->setKernel(checkK);
    Value * totalFailures = idb->getAccumulator("totalFailures");
    
    pxDriver.deallocateBuffers();
    idb->CreateRet(totalFailures);
    pxDriver.finalizeObject();
}

int main(int argc, char *argv[]) {
    cl::ParseCommandLineOptions(argc, argv);
    //codegen::SegmentSize = 1;
    ParabixDriver pxDriver("idisa_test");
    pipelineGen(pxDriver);
    
    int32_t fd1 = openFile(Operand1TestFile, llvm::outs());
    int32_t fd2 = openFile(Operand2TestFile, llvm::outs());
    
    auto idisaTestFunction = reinterpret_cast<IDISAtestFunctionType>(pxDriver.getMain());
    size_t failure_count = idisaTestFunction(fd1, fd2);
    if (!QuietMode) {
        if (failure_count == 0) {
            llvm::outs() << "Test success: " << TestOperation << "<" << TestFieldWidth << ">\n";
        } else {
            llvm::outs() << "Test failure: " << TestOperation << "<" << TestFieldWidth << "> failed " << failure_count << " tests!\n";
        }
    }
    close(fd1);
    close(fd2);
    return failure_count > 0;
}
