#ifndef REGIONSELECTIONKERNEL_H
#define REGIONSELECTIONKERNEL_H

#include <kernel/core/kernel.h>

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

    explicit RegionSelectionKernel(BuilderRef b, Starts starts, Ends ends, StreamSet * const regionSpans);

    explicit RegionSelectionKernel(BuilderRef b, Demarcators, Selectors selectors, StreamSet * const regionSpans);

    explicit RegionSelectionKernel(BuilderRef b, Starts starts, Ends ends, Selectors selectors, StreamSet * const regionSpans);

    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final;

protected:

    llvm::Value * getRegionStarts(BuilderRef b, llvm::Value * const offset) const;
    llvm::Value * getNumOfRegionStarts(BuilderRef b) const;
    llvm::Value * getRegionEnds(BuilderRef b, llvm::Value * const offset) const;
    llvm::Value * getNumOfRegionEnds(BuilderRef b) const;
    llvm::Value * getSelectorStream(BuilderRef b, llvm::Value * const offset) const;
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
