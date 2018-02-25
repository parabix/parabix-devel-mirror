/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef GREP_KERNEL_H
#define GREP_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace cc { class Alphabet; }
namespace kernel {

    
class RequiredStreams_UTF8 : public pablo::PabloKernel {
public:
    RequiredStreams_UTF8(const std::unique_ptr<kernel::KernelBuilder> & kb);
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
    ICGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::RE * const re_ast, std::vector<std::string> externals, std::vector<cc::Alphabet *> alphabets = {});
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    bool isCachable() const override { return true; }
protected:
    void generatePabloMethod() override;
    std::vector<std::string> mExternals;
    std::vector<cc::Alphabet *> mAlphabets;
};

class MatchedLinesKernel : public pablo::PabloKernel {
public:
    MatchedLinesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;    
};

class InvertMatchesKernel : public BlockOrientedKernel {
public:
    InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder);
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

class PopcountKernel : public pablo::PabloKernel {
public:
    PopcountKernel(const std::unique_ptr<kernel::KernelBuilder> & builder);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;    
};
    
}
#endif
