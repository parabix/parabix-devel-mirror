
#ifndef ICGREP_LZ4_MATCH_DETECTOR_H
#define ICGREP_LZ4_MATCH_DETECTOR_H

#include "kernels/kernel.h"
#include <map>
#include <vector>
#include <string>

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4MatchDetectorKernel : public SegmentOrientedKernel {
    public:
        LZ4MatchDetectorKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, unsigned blockSize = 4 * 1024 * 1024);
    protected:
        void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
    private:
        unsigned mLz4BlockSize;

        void appendOutput(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value *hasMatch);
        llvm::Value* detectMatch(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* start, llvm::Value* end);
    };
}




#endif //ICGREP_LZ4_MATCH_DETECTOR_H
