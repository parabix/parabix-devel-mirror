//
// Created by wxy325 on 2017/7/7.
//

#ifndef ICGREP_LZ4_EXTRACT_E1_M0_H
#define ICGREP_LZ4_EXTRACT_E1_M0_H

#include <kernels/sequential_kernel.h>
#include "../kernel.h"

namespace llvm {
    class Module;
    class Function;
    class BasicBlock;
    class Value;
}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    class LZ4ExtractEM0Kernel : public SequentialKernel {
    public:
        LZ4ExtractEM0Kernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::map<std::string, size_t>& inputIndexMap);

    protected:
        virtual void generateDoSequentialSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder);
    private:
        inline void generateIncreaseBlockDataIndex(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
        inline void generateRecordUncompressedBlock(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

        llvm::BasicBlock* generateHandleCompressedBlock(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

        inline llvm::Value* loadCurrentBlockData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::string& name);
    };
}


#endif //ICGREP_LZ4_EXTRACT_E1_M0_H
