#ifndef REGIONSELECTIONKERNEL_H
#define REGIONSELECTIONKERNEL_H

#include <kernels/core/kernel.h>

namespace kernel {

class RegionSelectionKernel final : public MultiBlockKernel {
public:

    #define STREAMSET_WITH_INDEX(NAME) \
        struct NAME { \
            StreamSet * const Stream; \
            const unsigned    Index; \
            NAME(StreamSet * stream, unsigned index) : Stream(stream), Index(index) { }; \
            NAME(std::pair<StreamSet *, unsigned> p) : Stream(std::get<0>(p)), Index(std::get<1>(p)) { }; \
        }
    STREAMSET_WITH_INDEX(Demarcators);
    STREAMSET_WITH_INDEX(Starts);
    STREAMSET_WITH_INDEX(Ends);
    STREAMSET_WITH_INDEX(Selectors);
    #undef STREAMSET_WITH_INDEX

    explicit RegionSelectionKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Starts starts, Ends ends, StreamSet * const regionSpans);

    explicit RegionSelectionKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Demarcators, Selectors selectors, StreamSet * const regionSpans);

    explicit RegionSelectionKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Starts starts, Ends ends, Selectors selectors, StreamSet * const regionSpans);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;

protected:

    llvm::Value * getRegionStarts(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const offset) const;
    llvm::Value * getNumOfRegionStarts(const std::unique_ptr<kernel::KernelBuilder> & b) const;
    llvm::Value * getRegionEnds(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const offset) const;
    llvm::Value * getNumOfRegionEnds(const std::unique_ptr<kernel::KernelBuilder> & b) const;
    llvm::Value * getSelectorStream(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const offset) const;
    LLVM_READNONE bool hasIndependentStartEndStreams() const;
    LLVM_READNONE bool hasSelectorStream() const;


private:

    const unsigned mStartStreamIndex;
    const unsigned mEndStreamIndex;
    const unsigned mSelectorStreamIndex;
    const bool mSelectorsAreAlignedWithRegionEnds;
    const bool mAlwaysExtendSelectedRegionsToRegionEnds;

};

}

#endif // REGIONSELECTIONKERNEL_H
