//
// Created by wxy325 on 2018/6/24.
//

#ifndef ICGREP_LZPARABIXCOMPRESSIONKERNEL_H
#define ICGREP_LZPARABIXCOMPRESSIONKERNEL_H

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

namespace kernel {
    struct MatchInfo {
        llvm::Value* matchOffset;
        llvm::Value* matchLength;
        llvm::Value* matchMask;
        llvm::Value* matchStart;
    };


    class LZParabixCompressionKernel : public SegmentOrientedKernel {
    public:
        LZParabixCompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b);
    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) override;
        size_t mLzParabixBlockSize;

        void encodeBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* inputCursor, llvm::Value* inputEndPos, llvm::Value* outputPos);

    private:
        void updateCache(const std::unique_ptr<KernelBuilder> &b, llvm::Value* i64BlockGlobalIndex, llvm::Value *initBlockGlobalIndex);
        inline llvm::Value* hashKey(const std::unique_ptr<KernelBuilder> &b, llvm::Value* v);
        MatchInfo extractMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos, llvm::Value *initBlockGlobalIndex, llvm::Value* inputEndPos);
        std::pair<llvm::Value*, llvm::Value*> getPossibleMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos);


        llvm::Value* appendLiteralSequence(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStart, llvm::Value* literalEnd, llvm::Value* outputPos);
        llvm::Value* appendMatchSequence(const std::unique_ptr<KernelBuilder> &b, const MatchInfo& matchInfo, llvm::Value* outputPos);

    };
}

#endif //ICGREP_LZPARABIXCOMPRESSIONKERNEL_H
