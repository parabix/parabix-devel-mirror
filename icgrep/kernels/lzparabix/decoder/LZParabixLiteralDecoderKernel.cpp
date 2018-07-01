//
// Created by wxy325 on 2018/6/30.
//

#include "LZParabixLiteralDecoderKernel.h"
#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>
#include <cstdint>


using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel {
    LZParabixLiteralDecoderKernel::LZParabixLiteralDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
            : SegmentOrientedKernel("LZParabixLiteralDecoderKernel",
            // Inputs
                                    {
                                            Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},

                                            // block data
                                            Binding{b->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1), AlwaysConsume()},
                                            Binding{b->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("blockStart"), AlwaysConsume()}


                                    },
            //Outputs
                                    {
                                            Binding{b->getStreamSetTy(8, 1), "literalBitStream", BoundedRate(0, 1)},
                                    },
            //Arguments
                                    {
                                            Binding{b->getSizeTy(), "fileSize"}
                                    },
                                    {},
            //Internal states:
                                    {
                                            Binding{b->getInt64Ty(), "outputPos"},

                                    })
    {
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZParabixLiteralDecoderKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {

        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = b->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = b->getProcessedItemCount("blockStart");
        Value * totalNumber = b->getAvailableItemCount("blockStart");

        Value * lz4BlockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", blockDataIndex);



        b->CreateCondBr(b->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        // ---- blockEndConBlock
        b->SetInsertPoint(blockEndConBlock);

        Value * blockStart = this->generateLoadInt64NumberInput(b, "blockStart", blockDataIndex);
        BasicBlock * processBlock = b->CreateBasicBlock("processBlock");

        b->CreateBr(processBlock);

        // ---- processBlock
        b->SetInsertPoint(processBlock);

        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        Value* newlyProduced = this->processBlock(b, blockStart);

        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));

        b->setProcessedItemCount("blockStart", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", lz4BlockEnd);
        b->setProducedItemCount("literalBitStream", b->CreateAdd(b->getProducedItemCount("literalBitStream"), newlyProduced));

        b->CreateBr(exitBlock);

        // ---- ExitBlock
        b->SetInsertPoint(exitBlock);
    }

    llvm::Value* LZParabixLiteralDecoderKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                  std::string inputBufferName, llvm::Value *globalOffset) {

        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    llvm::Value* LZParabixLiteralDecoderKernel::processBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockStart) {
        Value* blockBasePtr = b->getRawInputPointer("byteStream", blockStart);
        Value* literalLength = b->CreateLoad(b->CreatePointerCast(blockBasePtr, b->getInt32Ty()->getPointerTo()));
        literalLength = b->CreateZExtOrBitCast(literalLength, b->getSizeTy());
        Value* literalBasePtr = b->CreateGEP(blockBasePtr, b->getSize(4));


        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("literalBitStream", b->getSize(0)), b->getInt8PtrTy());

        Value* outputPos = b->getProducedItemCount("literalBitStream");
        Value* outputCapacity = b->getCapacity("literalBitStream");
        Value* outputPosRem = b->CreateURem(outputPos, outputCapacity);

        Value* remainingOutputSpace = b->CreateSub(outputCapacity, outputPosRem);


        Value* copyLength1 = b->CreateUMin(remainingOutputSpace, literalLength);
        Value* copyLength2 = b->CreateSub(literalLength, copyLength1);


        b->CreateMemCpy(
                b->CreateGEP(outputBasePtr, outputPosRem),
                literalBasePtr,
                copyLength1,
                1
        );

        b->CreateMemCpy(
                outputBasePtr,
                b->CreateGEP(literalBasePtr, copyLength1),
                copyLength2,
                1
        );

        return literalLength;
    }


}