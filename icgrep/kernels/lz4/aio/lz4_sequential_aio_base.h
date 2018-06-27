//
// Created by wxy325 on 2018/6/22.
//

#ifndef ICGREP_LZ4_AIO_BASE_H
#define ICGREP_LZ4_AIO_BASE_H

#include "kernels/kernel.h"
#include <string>
#include <map>
#include <vector>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel{

class LZ4SequentialAioBaseKernel : public SegmentOrientedKernel {
public:
    LZ4SequentialAioBaseKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::string&& kernelName, unsigned blockSize = 4 * 1024 * 1024);
protected:
    // ---- Constant
    const static unsigned int ACCELERATION_WIDTH = 64;

    // ---- Kernel Methods
    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;

    // ---- LZ4 Format Parsing
    virtual void processCompressedLz4Block(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                   llvm::Value *lz4BlockEnd);

    std::pair<std::pair<llvm::Value *, llvm::Value *>, llvm::Value *>
    doAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                   llvm::Value *blockEnd);


    virtual llvm::Value *processLz4Sequence(const std::unique_ptr<KernelBuilder> &b,
                                    llvm::Value *beginTokenPos, llvm::Value *lz4BlockEnd);

    std::pair<llvm::Value*, llvm::Value*> parseMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetBeginPos, llvm::Value* tokenValue);
    std::pair<llvm::Value*, llvm::Value*> parseMatchInfo2(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetBeginPos, llvm::Value* tokenValue);

    std::pair<llvm::Value *, llvm::Value *> noExtensionLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                            llvm::Value *currentTokenMarker,
                                                                            llvm::Value *currentExtenderValue,
                                                                            llvm::Value *tokenValue,
                                                                            llvm::Value *blockPosBase,
                                                                            llvm::Value *currentTokenLocalPos
    );

    std::pair<llvm::Value *, llvm::Value *> scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                         llvm::Value *currentTokenMarker,
                                                                         llvm::Value *currentExtenderValue,
                                                                         llvm::Value *tokenValue,
                                                                         llvm::Value *blockPosBase,
                                                                         llvm::Value *currentTokenLocalPos
    );

    std::pair<llvm::Value *, llvm::Value *> noExtensionMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                                          llvm::Value *matchOffsetEndMarker,
                                                                          llvm::Value *currentExtenderValue,
                                                                          llvm::Value *tokenValue,
                                                                          llvm::Value *blockPosBase
    );

    std::pair<llvm::Value *, llvm::Value *> scanThruMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                                       llvm::Value *matchOffsetEndMarker,
                                                                       llvm::Value *currentExtenderValue,
                                                                       llvm::Value *tokenValue,
                                                                       llvm::Value *blockPosBase
    );

    // ---- Basic Function
    llvm::Value *
    generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName,
                                 llvm::Value *globalOffset);
    llvm::Value *
    scanThru(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *from, llvm::Value *thru);

    // ---- Methods To Be Override


    virtual void doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                               llvm::Value *literalLength) = 0;

    virtual void doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                             llvm::Value *matchLength) = 0;

    virtual void storePendingOutput(const std::unique_ptr<KernelBuilder> &b) {};

    // Acceleration
    virtual void prepareAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginTokenPos) {};
    virtual void doAccelerationLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                           llvm::Value *literalLength) {this->doLiteralCopy(b, literalStart, literalLength);}
    virtual void doAccelerationMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                         llvm::Value *matchLength) {this->doMatchCopy(b, matchOffset, matchLength);}
    virtual void finishAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginTokenPos, llvm::Value* literalMask) {};

    virtual void setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) = 0;
};
}



#endif //ICGREP_LZ4_AIO_BASE_H
