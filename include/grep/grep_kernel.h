/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GREP_KERNEL_H
#define GREP_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <re/alphabet/alphabet.h>
#include <re/transforms/to_utf8.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace cc { class Alphabet; }
namespace kernel {


class UTF8_index : public pablo::PabloKernel {
public:
    UTF8_index(BuilderRef kb, StreamSet * Source, StreamSet * u8index);
protected:
    void generatePabloMethod() override;
};

enum class GrepCombiningType {None, Exclude, Include};
class GrepKernelOptions {
    friend class ICGrepKernel;
public:
    using Alphabets = std::vector<std::pair<std::shared_ptr<cc::Alphabet>, StreamSet *>>;
    GrepKernelOptions(const cc::Alphabet * codeUnitAlphabet = &cc::UTF8, re::EncodingTransformer * encodingTransformer = nullptr);
    void setIndexingTransformer(re::EncodingTransformer *, StreamSet * indexStream);
    void setSource(StreamSet * s);
    void setCombiningStream(GrepCombiningType t, StreamSet * toCombine);
    void setResults(StreamSet * r);

    void addExternal(std::string name, StreamSet * strm, int offset = 0, int lgth = 1, StreamSet * indexStrm = nullptr);

    void addAlphabet(std::shared_ptr<cc::Alphabet> a, StreamSet * basis);
    void setRE(re::RE * re);
    void setPrefixRE(re::RE * re);

protected:
    Bindings streamSetInputBindings();
    Bindings streamSetOutputBindings();
    Bindings scalarInputBindings();
    Bindings scalarOutputBindings();
    std::string makeSignature();

private:

    const cc::Alphabet *        mCodeUnitAlphabet;
    re::EncodingTransformer *   mEncodingTransformer;
    StreamSet *                 mSource = nullptr;
    StreamSet *                 mIndexStream = nullptr;
    GrepCombiningType           mCombiningType = GrepCombiningType::None;
    StreamSet *                 mCombiningStream = nullptr;
    StreamSet *                 mResults = nullptr;
    Bindings                    mExternals;
    Alphabets                   mAlphabets;
    re::RE *                    mRE = nullptr;
    re::RE *                    mPrefixRE = nullptr;
};


class ICGrepKernel : public pablo::PabloKernel {
public:
    ICGrepKernel(BuilderRef iBuilder,
                 std::unique_ptr<GrepKernelOptions> && options);
    llvm::StringRef getSignature() const override;
    bool hasSignature() const override { return true; }
    bool hasFamilyName() const override { return true; }
protected:
    void generatePabloMethod() override;
private:
    std::unique_ptr<GrepKernelOptions>  mOptions;
    std::string                         mSignature;
};

class MatchedLinesKernel : public pablo::PabloKernel {
public:
    MatchedLinesKernel(BuilderRef builder, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches);
protected:
    void generatePabloMethod() override;
};

class InvertMatchesKernel : public BlockOrientedKernel {
public:
    InvertMatchesKernel(BuilderRef b, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches);
private:
    void generateDoBlockMethod(BuilderRef iBuilder) override;
};

class FixedMatchPairsKernel : public pablo::PabloKernel {
public:
    FixedMatchPairsKernel(BuilderRef builder, unsigned length, StreamSet * MatchEnds, StreamSet * MatchPairs);
protected:
    void generatePabloMethod() override;
    unsigned mMatchLength;
};

class PopcountKernel : public pablo::PabloKernel {
public:
    PopcountKernel(BuilderRef builder, StreamSet * const toCount, Scalar * countResult);
protected:
    void generatePabloMethod() override;
};

class AbortOnNull final : public MultiBlockKernel {
public:
    AbortOnNull(BuilderRef, StreamSet * const InputStream, StreamSet * const OutputStream, Scalar * callbackObject);
private:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final;

};

/* Given a marker position P, a before-context B and and after-context A, a
   context span is a set of consecutive 1 bits from positions P-B to P+A.

   This kernel computes a coalesced context span stream for all markers in
   a given marker stream.   Coalesced spans occur when markers are separated
   by A + B positions or fewer. */

class ContextSpan final : public pablo::PabloKernel {
public:
    ContextSpan(BuilderRef b, StreamSet * const markerStream, StreamSet * const contextStream, unsigned before, unsigned after);
protected:
    void generatePabloMethod() override;
private:
    const unsigned          mBeforeContext;
    const unsigned          mAfterContext;
};

void GraphemeClusterLogic(const std::unique_ptr<ProgramBuilder> & P,
                          re::UTF16_Transformer * t,
                          StreamSet * Source, StreamSet * U8index, StreamSet * GCBstream);

}
#endif
