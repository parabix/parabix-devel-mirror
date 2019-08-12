/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GREP_KERNEL_H
#define GREP_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <re/alphabet/alphabet.h>

namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace cc { class Alphabet; }
namespace kernel {


class UTF8_index : public pablo::PabloKernel {
public:
    UTF8_index(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * Source, StreamSet * u8index);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

enum class GrepCombiningType {None, Exclude, Include};
class GrepKernelOptions {
    friend class ICGrepKernel;
public:
    using Alphabets = std::vector<std::pair<std::shared_ptr<cc::Alphabet>, StreamSet *>>;
    GrepKernelOptions() :
        mIndexingAlphabet(&cc::Byte),
        mCombiningType(GrepCombiningType::None),
        mPrefixRE(nullptr) {}
    void setIndexingAlphabet(const cc::Alphabet * a);
    void setSource(StreamSet * s);
    void setCombiningStream(GrepCombiningType t, StreamSet * toCombine);
    void setResults(StreamSet * r);

    void addExternal(std::string name, StreamSet * strm) {
        mExternals.emplace_back(name, strm, FixedRate(), ZeroExtended());
    }

    template <typename ... Args>
    void addExternal(std::string name, StreamSet * strm, Args ... args) {
        std::initializer_list<Attribute> attrs{std::forward<Args>(args)...};
        mExternals.emplace_back(name, strm, FixedRate(), attrs);
    }

    void addAlphabet(std::shared_ptr<cc::Alphabet> a, StreamSet * basis);
    void setRE(re::RE * re);
    void setPrefixRE(re::RE * re);

protected:
    Bindings streamSetInputBindings();
    Bindings streamSetOutputBindings();
    Bindings scalarInputBindings();
    Bindings scalarOutputBindings();
    std::string getSignature();

private:
    const cc::Alphabet * mIndexingAlphabet;
    StreamSet * mSource;
    GrepCombiningType mCombiningType;
    StreamSet * mCombiningStream;
    StreamSet * mResults;
    Bindings mExternals;
    Alphabets mAlphabets;
    re::RE * mRE;
    re::RE * mPrefixRE;
    std::string     mSignature;
};


class ICGrepKernel : public pablo::PabloKernel {
public:
    ICGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
                 std::unique_ptr<GrepKernelOptions> options);
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) const override;
    bool isCachable() const override { return true; }
    bool hasFamilyName() const override { return true; }
protected:
    void generatePabloMethod() override;
    std::unique_ptr<GrepKernelOptions> mOptions;
};

struct ByteBitGrepSignature {
    ByteBitGrepSignature(re::RE * prefix, re::RE * suffix);
protected:
    re::RE * const  mPrefixRE;
    re::RE * const  mSuffixRE;
    std::string     mSignature;
};


class ByteBitGrepKernel : public ByteBitGrepSignature, public pablo::PabloKernel {
    using Externals = std::vector<std::pair<std::string, StreamSet *>>;
public:
    ByteBitGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::RE * const prefix, re::RE * const suffix, StreamSet * const Source, StreamSet * const MatchResults,
                      const Externals externals = {});
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) const override;
    bool isCachable() const override { return true; }
    bool hasFamilyName() const override { return true; }
private:
    static Bindings makeInputBindings(StreamSet * const source, const Externals & externals);
protected:
    void generatePabloMethod() override;
};

class MatchedLinesKernel : public pablo::PabloKernel {
public:
    MatchedLinesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class InvertMatchesKernel : public BlockOrientedKernel {
public:
    InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

class PopcountKernel : public pablo::PabloKernel {
public:
    PopcountKernel(const std::unique_ptr<kernel::KernelBuilder> & builder, StreamSet * const toCount, Scalar * countResult);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class AbortOnNull final : public MultiBlockKernel {
public:
    AbortOnNull(const std::unique_ptr<kernel::KernelBuilder> &, StreamSet * const InputStream, StreamSet * const OutputStream, Scalar * callbackObject);
private:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;

};

/* Given a marker position P, a before-context B and and after-context A, a
   context span is a set of consecutive 1 bits from positions P-B to P+A.

   This kernel computes a coalesced context span stream for all markers in
   a given marker stream.   Coalesced spans occur when markers are separated
   by A + B positions or fewer. */

class ContextSpan final : public pablo::PabloKernel {
public:
    ContextSpan(const std::unique_ptr<KernelBuilder> & b, StreamSet * const markerStream, StreamSet * const contextStream, unsigned before, unsigned after);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    const unsigned          mBeforeContext;
    const unsigned          mAfterContext;
};

}
#endif
