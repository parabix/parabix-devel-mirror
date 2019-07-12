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
#include <kernel/core/kernel_builder.h>
#include <kernel/core/idisa_target.h>
#include <kernel/core/streamset.h>
#include <kernel/util/source_kernel.h>
#include <kernel/util/hex_convert.h>
#include <kernel/util/s2p_kernel.h>
#include <kernel/util/stdout_kernel.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

using namespace llvm;
using namespace kernel;

static cl::OptionCategory testFlags("Command Flags", "test options");

static cl::opt<std::string> TestOperation(cl::Positional, cl::desc("Operation to test"), cl::Required, cl::cat(testFlags));

static cl::opt<int> TestFieldWidth(cl::Positional, cl::desc("Test field width (default 64)."), cl::init(64), cl::Required, cl::cat(testFlags));

static cl::opt<std::string> Operand1TestFile(cl::Positional, cl::desc("Operand 1 data file."), cl::Required, cl::cat(testFlags));
static cl::opt<std::string> Operand2TestFile(cl::Positional, cl::desc("Operand 2 data file."), cl::Required, cl::cat(testFlags));
static cl::opt<std::string> TestOutputFile("o", cl::desc("Test output file."), cl::cat(testFlags));
static cl::opt<bool> QuietMode("q", cl::desc("Suppress output, set the return code only."), cl::cat(testFlags));
static cl::opt<int> ShiftLimit("ShiftLimit", cl::desc("Upper limit for the shift operand (2nd operand) of sllv, srlv, srav."), cl::init(0));
static cl::opt<int> Immediate("i", cl::desc("Immediate value for mvmd_dslli"), cl::init(1));

class ShiftLimitKernel : public BlockOrientedKernel {
public:
    ShiftLimitKernel(const std::unique_ptr<KernelBuilder> & b, unsigned fw, unsigned limit, StreamSet * input, StreamSet * output);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) override;
private:
    const unsigned mTestFw;
    const unsigned mShiftLimit;
};

ShiftLimitKernel::ShiftLimitKernel(const std::unique_ptr<KernelBuilder> & b, unsigned fw, unsigned limit, StreamSet *input, StreamSet *output)
: BlockOrientedKernel(b, "shiftLimit" + std::to_string(fw) + "_" + std::to_string(limit),
                              {Binding{"shiftOperand", input}},
                              {Binding{"limitedShift", output}},
                              {}, {}, {}),
mTestFw(fw), mShiftLimit(limit) {}

void ShiftLimitKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) {
    Type * fwTy = kb->getIntNTy(mTestFw);
    Constant * const ZeroConst = kb->getSize(0);
    Value * shiftOperand = kb->loadInputStreamBlock("shiftOperand", ZeroConst);
    unsigned fieldCount = kb->getBitBlockWidth()/mTestFw;
    Value * limited = kb->simd_umin(mTestFw, shiftOperand, ConstantVector::getSplat(fieldCount, ConstantInt::get(fwTy, mShiftLimit)));
    kb->storeOutputStreamBlock("limitedShift", ZeroConst, limited);
}

class IdisaBinaryOpTestKernel : public MultiBlockKernel {
public:
    IdisaBinaryOpTestKernel(const std::unique_ptr<KernelBuilder> &b, std::string idisa_op, unsigned fw, unsigned imm,
                            StreamSet * Operand1, StreamSet * Operand2, StreamSet * result);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const std::string mIdisaOperation;
    const unsigned mTestFw;
    const unsigned mImmediateShift;
};

IdisaBinaryOpTestKernel::IdisaBinaryOpTestKernel(const std::unique_ptr<KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm,
                                                 StreamSet *Operand1, StreamSet *Operand2, StreamSet *result)
: MultiBlockKernel(b, idisa_op + std::to_string(fw) + "_test",
     {Binding{"operand1", Operand1}, Binding{"operand2", Operand2}},
     {Binding{"result", result}},
     {}, {}, {}),
mIdisaOperation(std::move(idisa_op)), mTestFw(fw), mImmediateShift(imm) {}

void IdisaBinaryOpTestKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
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
    } else if (mIdisaOperation == "simd_ne") {
        result = kb->simd_ne(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_gt") {
        result = kb->simd_gt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ugt") {
        result = kb->simd_ugt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ge") {
        result = kb->simd_ge(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_uge") {
        result = kb->simd_uge(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_lt") {
        result = kb->simd_lt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_le") {
        result = kb->simd_le(mTestFw, operand1, operand2);
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
    } else if (mIdisaOperation == "hsimd_packus") {
        result = kb->hsimd_packus(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "hsimd_packss") {
        result = kb->hsimd_packss(mTestFw, operand1, operand2);
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

class IdisaBinaryOpCheckKernel : public BlockOrientedKernel {
public:
    IdisaBinaryOpCheckKernel(const std::unique_ptr<KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm,
                             StreamSet * Operand1, StreamSet * Operand2, StreamSet * result,
                             StreamSet * expected, Scalar * failures);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) override;
private:
    const std::string mIdisaOperation;
    const unsigned mTestFw;
    const unsigned mImmediateShift;
};

IdisaBinaryOpCheckKernel::IdisaBinaryOpCheckKernel(const std::unique_ptr<KernelBuilder> & b, std::string idisa_op, unsigned fw, unsigned imm,
                                                   StreamSet *Operand1, StreamSet *Operand2, StreamSet *result,
                                                   StreamSet *expected, Scalar *failures)
: BlockOrientedKernel(b, idisa_op + std::to_string(fw) + "_check" + std::to_string(QuietMode),
                           {Binding{"operand1", Operand1},
                            Binding{"operand2", Operand2},
                            Binding{"test_result", result}},
                           {Binding{"expected_result", expected}},
                           {}, {Binding{"totalFailures", failures}}, {}),
mIdisaOperation(idisa_op), mTestFw(fw), mImmediateShift(imm) {}

void IdisaBinaryOpCheckKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) {
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
                } else if (mIdisaOperation == "simd_ne") {
                    expected = kb->CreateSExt(kb->CreateICmpNE(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_gt") {
                    expected = kb->CreateSExt(kb->CreateICmpSGT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_ge") {
                    expected = kb->CreateSExt(kb->CreateICmpSGE(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_ugt") {
                    expected = kb->CreateSExt(kb->CreateICmpUGT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_uge") {
                    expected = kb->CreateSExt(kb->CreateICmpUGE(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_lt") {
                    expected = kb->CreateSExt(kb->CreateICmpSLT(operand1, operand2), fwTy);
                } else if (mIdisaOperation == "simd_le") {
                    expected = kb->CreateSExt(kb->CreateICmpSLE(operand1, operand2), fwTy);
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
            } else if (mIdisaOperation == "hsimd_packus") {
                Value * zeroes = ConstantInt::getNullValue(operand1->getType());
                operand1 = kb->CreateSelect(kb->CreateICmpSLT(operand1, zeroes), zeroes, operand1);
                operand2 = kb->CreateSelect(kb->CreateICmpSLT(operand2, zeroes), zeroes, operand2);
                Value * testVal = ConstantInt::get(kb->getContext(), APInt::getLowBitsSet(mTestFw, mTestFw/2));
                operand1 = kb->CreateSelect(kb->CreateICmpSGT(operand1, testVal), testVal, operand1);
                operand2 = kb->CreateSelect(kb->CreateICmpSGT(operand2, testVal), testVal, operand2);
                expectedBlock = kb->mvmd_insert(mTestFw/2, expectedBlock, operand1, i);
                expectedBlock = kb->bitCast(kb->mvmd_insert(mTestFw/2, expectedBlock, operand2, fieldCount + i));
            } else if (mIdisaOperation == "hsimd_packss") {
                Value * testVal = ConstantInt::get(kb->getIntNTy(mTestFw), (1 << (mTestFw/2 - 1)) - 1);
                operand1 = kb->CreateSelect(kb->CreateICmpSGT(operand1, testVal), testVal, operand1);
                operand2 = kb->CreateSelect(kb->CreateICmpSGT(operand2, testVal), testVal, operand2);
                testVal = kb->CreateNot(testVal);
                operand1 = kb->CreateSelect(kb->CreateICmpSLT(operand1, testVal), testVal, operand1);
                operand2 = kb->CreateSelect(kb->CreateICmpSLT(operand2, testVal), testVal, operand2);
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
    Value * failures = kb->simd_ugt(mTestFw, kb->simd_xor(resultBlock, expectedBlock), kb->allZeroes());
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

StreamSet * readHexToBinary(std::unique_ptr<ProgramBuilder> & P, const std::string & fd) {
    StreamSet * const hexStream = P->CreateStreamSet(1, 8);
    Scalar * const fileDecriptor = P->getInputScalar(fd);
    P->CreateKernelCall<MMapSourceKernel>(fileDecriptor, hexStream);
    StreamSet * const bitStream = P->CreateStreamSet(1, 1);
    P->CreateKernelCall<HexToBinary>(hexStream, bitStream);
    return bitStream;
}

inline StreamSet * applyShiftLimit(std::unique_ptr<ProgramBuilder> & P, StreamSet * const input) {
    if (ShiftLimit > 0) {
        StreamSet * output = P->CreateStreamSet(1, 1);
        P->CreateKernelCall<ShiftLimitKernel>(TestFieldWidth, ShiftLimit, input, output);
        return output;
    }
    return input;
}

IDISAtestFunctionType pipelineGen(CPUDriver & pxDriver) {

    auto & b = pxDriver.getBuilder();

    Type * const sizeTy = b->getSizeTy();
    Type * const int32Ty = b->getInt32Ty();

    Bindings inputs;
    inputs.emplace_back(int32Ty, "operand1FileDecriptor");
    inputs.emplace_back(int32Ty, "operand2FileDecriptor");
    if (!TestOutputFile.empty()) {
        inputs.emplace_back(b->getInt8PtrTy(), "outputFileName");
    }

    auto P = pxDriver.makePipeline(std::move(inputs), {Binding{sizeTy, "totalFailures"}});


    StreamSet * const Operand1BitStream = readHexToBinary(P, "operand1FileDecriptor");
    StreamSet * const Operand2BitStream = applyShiftLimit(P, readHexToBinary(P, "operand2FileDecriptor"));

    StreamSet * const ResultBitStream = P->CreateStreamSet(1, 1);

    P->CreateKernelCall<IdisaBinaryOpTestKernel>(TestOperation, TestFieldWidth, Immediate
                                                 , Operand1BitStream, Operand2BitStream
                                                 , ResultBitStream);

    StreamSet * ExpectedResultBitStream = P->CreateStreamSet(1, 1);

    P->CreateKernelCall<IdisaBinaryOpCheckKernel>(TestOperation, TestFieldWidth, Immediate
                                                 , Operand1BitStream, Operand2BitStream, ResultBitStream
                                                 , ExpectedResultBitStream, P->getOutputScalar("totalFailures"));

    if (!TestOutputFile.empty()) {
        StreamSet * ResultHexStream = P->CreateStreamSet(1, 8);
        P->CreateKernelCall<BinaryToHex>(ResultBitStream, ResultHexStream);
        Scalar * outputFileName = P->getInputScalar("outputFileName");
        P->CreateKernelCall<FileSink>(outputFileName, ResultHexStream);
    }

    return reinterpret_cast<IDISAtestFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    cl::ParseCommandLineOptions(argc, argv);
    //codegen::SegmentSize = 1;
    CPUDriver pxDriver("idisa_test");
    auto idisaTestFunction = pipelineGen(pxDriver);

    const int32_t fd1 = openFile(Operand1TestFile, llvm::outs());
    const int32_t fd2 = openFile(Operand2TestFile, llvm::outs());
    const size_t failure_count = idisaTestFunction(fd1, fd2);
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
