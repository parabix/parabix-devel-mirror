
#include "lz4_match_detector.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <llvm/Support/raw_ostream.h>
#include <kernels/core/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel {
    LZ4MatchDetectorKernel::LZ4MatchDetectorKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned blockSize)
            : SegmentOrientedKernel(b, "LZ4MatchDetectorKernel",
// Inputs
                                    {
                                            Binding{b->getStreamSetTy(1), "matches", BoundedRate(0, 1)},
                                            Binding{b->getStreamSetTy(1), "linebreak", RateEqualTo("matches")}
                                    },
//Outputs
                                    {

                                            Binding{b->getStreamSetTy(1, 8), "hasMatches", BoundedRate(0, 1)}
                                    },
//Arguments
                                    {
                                    },
                                    {},
//Internal states:
                                    {
                                    }),
              mLz4BlockSize(blockSize)
    {
        setStride(blockSize);
    }

    void LZ4MatchDetectorKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        Constant* blockSize = b->getSize(mLz4BlockSize);

        Value* processed = b->getProcessedItemCount("matches");
        Value* total = b->getAvailableItemCount("matches");

        Value* available = b->CreateSub(total, processed);

        Value* enoughData = b->CreateOr(
                mIsFinal,
                b->CreateICmpUGE(available, blockSize)
        );

        BasicBlock* processBlock = b->CreateBasicBlock("processBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");


        b->CreateCondBr(enoughData, processBlock, exitBlock);
        b->SetInsertPoint(processBlock);

        Value* startPos = processed;
        Value* endPos = b->CreateAdd(processed, blockSize);
        endPos = b->CreateUMin(endPos, total);
        Value* hasMatch = detectMatch(b, startPos, endPos);
        appendOutput(b, hasMatch);
        b->setProcessedItemCount("matches", endPos);
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->setTerminationSignal(mIsFinal);
    }

    llvm::Value* LZ4MatchDetectorKernel::detectMatch(const std::unique_ptr<KernelBuilder> & b, llvm::Value* start, llvm::Value* end) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Constant* SIZE_63 = b->getSize(63);
        Constant* SIZE_64 = b->getSize(64);

        Value* bufferCapacity = b->getCapacity("matches");
        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("matches", b->getSize(0)), b->getInt64Ty()->getPointerTo());

        Value* startRem = b->CreateURem(start, bufferCapacity);
        Value* endRem = b->CreateSub(end, b->CreateSub(start, startRem));
        Value* startI64BlockIndex = b->CreateUDiv(startRem, SIZE_64);

        Value* endI64BlockIndex = b->CreateUDiv(b->CreateAdd(endRem, SIZE_63), SIZE_64);

        Value* startPtr = b->CreateGEP(inputBasePtr, startI64BlockIndex);
        b->CallPrintInt("startI64BlockIndex", startI64BlockIndex);
        Value* endPtr = b->CreateGEP(inputBasePtr, endI64BlockIndex);

        BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
        BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        b->CreateBr(conBlock);

        b->SetInsertPoint(conBlock);

        PHINode* currentPtr = b->CreatePHI(startPtr->getType(), 2);
        PHINode* hasMatch = b->CreatePHI(b->getInt1Ty(), 2);
        currentPtr->addIncoming(startPtr, entryBlock);
        hasMatch->addIncoming(b->getInt1(false), entryBlock);

        Value* shouldContinue = b->CreateAnd(b->CreateNot(hasMatch), b->CreateICmpNE(currentPtr, endPtr));

        b->CreateCondBr(shouldContinue, bodyBlock, exitBlock);

        b->SetInsertPoint(bodyBlock);
        Value* currentValue = b->CreateLoad(currentPtr);
//        b->CallPrintInt("currentValue", currentValue);
        Value* m = b->CreateICmpNE(currentValue, b->getInt64(0));

        currentPtr->addIncoming(b->CreateGEP(currentPtr, b->getSize(1)), b->GetInsertBlock());
        hasMatch->addIncoming(b->CreateOr(hasMatch, m), b->GetInsertBlock());
        b->CreateBr(conBlock);

        b->SetInsertPoint(exitBlock);
        return hasMatch;
    }

    void LZ4MatchDetectorKernel::appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, Value * const hasMatch) {
        iBuilder->CallPrintInt("hasMatch", hasMatch);
        Value * const offset = iBuilder->getProducedItemCount("hasMatches");
        iBuilder->CreateStore(iBuilder->CreateZExtOrBitCast(hasMatch, iBuilder->getInt8Ty()), iBuilder->getRawOutputPointer("hasMatches", offset));
        iBuilder->setProducedItemCount("hasMatches", iBuilder->CreateAdd(offset, iBuilder->getSize(1)));
    }
}
