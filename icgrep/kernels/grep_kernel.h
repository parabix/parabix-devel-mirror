/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GREP_KERNEL_H
#define GREP_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <cc/alphabet.h>

namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace cc { class Alphabet; }
namespace kernel {

    
class UnicodeNonFinalKernel : public pablo::PabloKernel {
public:
    UnicodeNonFinalKernel(const std::unique_ptr<kernel::KernelBuilder> & kb);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class UnicodeLineBreakKernel : public pablo::PabloKernel {
public:
    UnicodeLineBreakKernel(const std::unique_ptr<kernel::KernelBuilder> & kb);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class RequiredStreams_UTF8 : public pablo::PabloKernel {
public:
    RequiredStreams_UTF8(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * LineFeedStream, StreamSet * RequiredStreams, StreamSet * UnicodeLB);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class RequiredStreams_UTF16 : public pablo::PabloKernel {
public:
    RequiredStreams_UTF16(const std::unique_ptr<kernel::KernelBuilder> & kb);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};


struct ICGrepSignature {
    ICGrepSignature(re::RE * re_ast);
protected:
    re::RE * const  mRE;
    std::string     mSignature;
};

    
class ICGrepKernel : public ICGrepSignature, public pablo::PabloKernel {
public:

    using Externals = std::vector<std::pair<std::string, StreamSet *>>;
    using Alphabets = std::vector<std::pair<std::shared_ptr<cc::Alphabet>, StreamSet *>>;

    ICGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
                 re::RE * const re_ast, StreamSet * const BasisBits, StreamSet * const MatchResults,
                 const Externals externals = {}, const Alphabets alphabets = {},
                 const cc::BitNumbering basisSetNumbering = cc::BitNumbering::LittleEndian,
                 const bool cachable = true);
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) override;
    bool isCachable() const override { return mIsCachable; }
    bool hasFamilyName() const override { return true; }
protected:
    void generatePabloMethod() override;
private:
    static Bindings makeInputBindings(StreamSet * const basis, const Externals & externals, const Alphabets & alphabets);
protected:
    const Externals mExternals;
    const Alphabets mAlphabets;
    const cc::BitNumbering mBasisSetNumbering;
    const bool mIsCachable;
};

struct ByteGrepSignature {
    ByteGrepSignature(re::RE * re);
protected:
    re::RE * const  mRE;
    std::string     mSignature;
};


class ByteGrepKernel : public ByteGrepSignature, public pablo::PabloKernel {
public:
    ByteGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::RE * const re, std::vector<Binding> inputSets, StreamSet * matches);
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) override;
    bool isCachable() const override { return true; }
    bool hasFamilyName() const override { return true; }
protected:
    void generatePabloMethod() override;
};
    
struct ByteBitGrepSignature {
    ByteBitGrepSignature(re::RE * prefix, re::RE * suffix);
protected:
    re::RE * const  mPrefixRE;
    re::RE * const  mSuffixRE;
    std::string     mSignature;
};

    
class ByteBitGrepKernel : public ByteBitGrepSignature, public pablo::PabloKernel {
public:
    ByteBitGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::RE * const prefix, re::RE * const suffix, std::vector<Binding> inputSets, StreamSet * matches);
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) override;
    bool isCachable() const override { return true; }
    bool hasFamilyName() const override { return true; }
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
    InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches);
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

}
#endif
