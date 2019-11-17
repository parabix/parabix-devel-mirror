#include <grep/nested_grep_engine.h>
#include <grep/regex_passes.h>
#include <re/unicode/casing.h>
#include <re/transforms/exclude_CC.h>
#include <re/transforms/to_utf8.h>
#include <re/unicode/re_name_resolve.h>
#include <kernel/io/source_kernel.h>
#include <kernel/basis/s2p_kernel.h>
#include <re/cc/cc_kernel.h>
#include <kernel/scan/scanmatchgen.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/core/kernel_builder.h>

#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace llvm;

namespace grep {

class CopyBreaksToMatches final : public MultiBlockKernel {
public:

    CopyBreaksToMatches(BuilderRef b,
               StreamSet * const BasisBits,
               StreamSet * const u8index,
               StreamSet * const breaks,
               StreamSet * const matches)
    : MultiBlockKernel(b
                       , "gitignoreC"
                       // inputs
                       , {{"BasisBits", BasisBits}, {"u8index", u8index}, {"breaks", breaks}}
                       // outputs
                       , {{"matches", matches, FixedRate(), Add1()}}
                       // scalars
                       , {}, {}, {}) { }


    bool hasFamilyName() const override { return true; }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

protected:
    void generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
        PointerType * const int8PtrTy = b->getInt8PtrTy();
        Value * const processed = b->getProcessedItemCount("breaks");
        Value * const source = b->CreatePointerCast(b->getRawInputPointer("breaks", processed), int8PtrTy);
        Value * const produced = b->getProducedItemCount("matches");
        Value * const target = b->CreatePointerCast(b->getRawOutputPointer("matches", produced), int8PtrTy);
        const auto blockSize = b->getBitBlockWidth() / 8;
        Value * const toCopy = b->CreateMul(numOfStrides, b->getSize(blockSize));
        b->CreateMemCpy(target, source, toCopy, blockSize);
    }

};

class NestedGrepPipelineKernel : public PipelineKernel {
public:
    NestedGrepPipelineKernel(BaseDriver & driver,

                             StreamSet * const BasisBits,
                             StreamSet * const u8index,
                             StreamSet * const breaks,
                             StreamSet * const matches,

                             Kernel * const outerKernel,
                             const re::PatternVector & patterns,
                             const bool caseInsensitive,
                             re::CC * const breakCC)
        : PipelineKernel(driver
                         // signature
                         , [&]() -> std::string {
                            std::string tmp;
                            raw_string_ostream out(tmp);
                            out << "gitignore" << (outerKernel == nullptr ? 'R' : 'N');
                            out.write_hex(patterns.size());
                            out.flush();
                            return tmp;
                         }()
                         // num of threads
                         , 1
                         // make kernel list
                         , [&]() -> Kernels {
                             Kernels kernels;

                             assert(BasisBits && u8index && breaks && matches);

                             const auto n = patterns.size();
                             auto & b = driver.getBuilder();

                             assert ("logic error: outer kernel should have been used directly when n == 0" && n > 0);

                             StreamSet * resultSoFar = breaks;

                             if (LLVM_LIKELY(outerKernel != nullptr)) {
                                 resultSoFar = driver.CreateStreamSet();
                                 outerKernel->setOutputStreamSet("matches", resultSoFar);
                                 // if we already constructed the outer kernel through nesting,
                                 // it may not be necessary to add it back to the execution engine.
                                 driver.addKernel(outerKernel);
                                 kernels.push_back(outerKernel);
                             }

                             for (unsigned i = 0; i != n; i++) {
                                 StreamSet * MatchResults = nullptr;
                                 if (LLVM_UNLIKELY(i == (n - 1UL))) {
                                     MatchResults = matches;
                                 } else {
                                     MatchResults = driver.CreateStreamSet();
                                 }

                                 auto options = make_unique<GrepKernelOptions>();

                                 auto r = resolveCaseInsensitiveMode(patterns[i].second, caseInsensitive);
                                 r = regular_expression_passes(r);
                                 r = re::exclude_CC(r, breakCC);
                                 r = resolveAnchors(r, breakCC);
                                 r = toUTF8(r);

                                 options->setRE(r);
                                 options->setSource(BasisBits);
                                 options->setResults(MatchResults);
                                 // check if we need to combine the current result with the new set of matches
                                 const bool exclude = (patterns[i].first == re::PatternKind::Exclude);
                                 if (i || outerKernel || exclude) {
                                     options->setCombiningStream(exclude ? GrepCombiningType::Exclude : GrepCombiningType::Include, resultSoFar);
                                 }
                                 options->addExternal("UTF8_index", u8index);
                                 Kernel * const matcher = new ICGrepKernel(b, std::move(options));
                                 driver.addKernel(matcher);
                                 kernels.push_back(matcher);
                                 resultSoFar = MatchResults;
                             }
                             assert (resultSoFar == matches);

                             driver.generateUncachedKernels();
                             return kernels;
                         }()
                         // called functions
                         , {}
                         // stream inputs
                         , {{"basis", BasisBits}, {"u8index", u8index}, {"breaks", breaks}}
                         // stream outputs
                         , {{"matches", matches}}
                         // scalars
                         , {}, {}) {
        setStride(driver.getBuilder()->getBitBlockWidth());
        addAttribute(InternallySynchronized());
    }

    bool hasFamilyName() const override { return true; }

};

////PipelineKernel(BuilderRef b,
////               std::string && signature,
////               const unsigned numOfThreads,
////               Kernels && kernels, CallBindings && callBindings,
////               Bindings && stream_inputs, Bindings && stream_outputs,
////               Bindings && scalar_inputs, Bindings && scalar_outputs);

//class RootGrepPipelineKernel : public PipelineKernel {

//    RootGrepPipelineKernel(BaseDriver & driver,
//                             BuilderRef b,

//                             Scalar * const buffer,
//                             Scalar * const length,
//                             Scalar * const accumulator) {



//    }

//};

NestedInternalSearchEngine::NestedInternalSearchEngine(BaseDriver & driver)
: mGrepRecordBreak(GrepRecordBreakKind::LF)
, mCaseInsensitive(false)
, mGrepDriver(driver)
, mNumOfThreads(codegen::ThreadNum)
, mBreakCC(nullptr)
, mNested(1, nullptr) {

}

void NestedInternalSearchEngine::push(const re::PatternVector & patterns) {
    // If we have no patterns and this is the "root" pattern,
    // we'll still need an empty gitignore kernel even if it
    // just returns the record break stream for input.
    // Otherwise just reuse the parent kernel.

    assert (mBreakCC && mBasisBits && mU8index && mBreaks && mMatches);

    Kernel * kernel = nullptr;

    if (LLVM_UNLIKELY(patterns.empty())) {
        if (LLVM_LIKELY(mNested.size() > 1)) {
            mNested.push_back(mNested.back());
            return;
        } else {
            kernel = new CopyBreaksToMatches(mGrepDriver.getBuilder(),
                                             mBasisBits, mU8index, mBreaks,
                                             mMatches);
        }
    } else {
        kernel = new NestedGrepPipelineKernel(mGrepDriver,
                                              mBasisBits, mU8index, mBreaks,
                                              mMatches,
                                              mNested.back(), // outer kernel
                                              patterns, mCaseInsensitive, mBreakCC);
    }
    mGrepDriver.addKernel(kernel);
    mNested.push_back(kernel);
}

void NestedInternalSearchEngine::pop() {
    assert (mNested.size() > 1);
    mNested.pop_back();
    assert (mMainMethod.size() > 0);
    mMainMethod.pop_back();
}

void NestedInternalSearchEngine::init() {

    if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        mBreakCC = re::makeByte(0x0);
    } else {// if (mGrepRecordBreak == GrepRecordBreakKind::LF)
        mBreakCC = re::makeByte(0x0A);
    }

    mBasisBits = mGrepDriver.CreateStreamSet(8);
    mU8index = mGrepDriver.CreateStreamSet();
    mBreaks = mGrepDriver.CreateStreamSet();
    mMatches = mGrepDriver.CreateStreamSet();

}

void NestedInternalSearchEngine::grepCodeGen() {
    auto & b = mGrepDriver.getBuilder();

    // TODO: we should be able to avoid constructing the main pipeline if there is a way to
    // pass the information for the nested kernel address in through the "main" function.

    assert (mBreakCC && mBasisBits && mU8index && mBreaks && mMatches);

    Scalar * const buffer = mGrepDriver.CreateScalar(b->getInt8PtrTy());
    Scalar * const length = mGrepDriver.CreateScalar(b->getSizeTy());
    Scalar * const accumulator = mGrepDriver.CreateScalar(b->getIntAddrTy());

    auto E = mGrepDriver.makePipeline({Binding{"buffer", buffer},
        Binding{"length", length},
        Binding{"accumulator", accumulator}});
    E->setNumOfThreads(mNumOfThreads);

    StreamSet * const ByteStream = E->CreateStreamSet(1, 8);
    E->CreateKernelCall<MemorySourceKernel>(buffer, length, ByteStream);

    const auto RBname = (mGrepRecordBreak == GrepRecordBreakKind::Null) ? "Null" : "LF";
    E->CreateKernelCall<S2PKernel>(ByteStream, mBasisBits);
    E->CreateKernelCall<CharacterClassKernelBuilder>(RBname, std::vector<re::CC *>{mBreakCC}, mBasisBits, mBreaks);
    E->CreateKernelCall<UTF8_index>(mBasisBits, mU8index);

    assert (mNested.size() > 1 && mNested.back());
    E->AddKernelCall(mNested.back());

//    if (MatchCoordinateBlocks > 0) {
//        StreamSet * MatchCoords = E->CreateStreamSet(3, sizeof(size_t) * 8);
//        E->CreateKernelCall<MatchCoordinatesKernel>(matches, RecordBreakStream, MatchCoords, MatchCoordinateBlocks);
//        Kernel * const matchK = E->CreateKernelCall<MatchReporter>(ByteStream, MatchCoords, callbackObject);
//        mGrepDriver.LinkFunction(matchK, "accumulate_match_wrapper", accumulate_match_wrapper);
//        mGrepDriver.LinkFunction(matchK, "finalize_match_wrapper", finalize_match_wrapper);
//    } else {
        Kernel * const scanMatchK = E->CreateKernelCall<ScanMatchKernel>(mMatches, mBreaks, ByteStream, accumulator, 4); // ScanMatchBlocks
        mGrepDriver.LinkFunction(scanMatchK, "accumulate_match_wrapper", accumulate_match_wrapper);
        mGrepDriver.LinkFunction(scanMatchK, "finalize_match_wrapper", finalize_match_wrapper);
//    }

    mMainMethod.push_back(E->compile());
}

void NestedInternalSearchEngine::doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum) {
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length, MatchAccumulator *);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod.back());
    f(search_buffer, bufferLength, &accum);
}

//NestedInternalSearchEngine::NestedInternalSearchEngine(const std::unique_ptr<grep::GrepEngine> & engine)
//: NestedInternalSearchEngine(engine->mGrepDriver) {

//}

NestedInternalSearchEngine::~NestedInternalSearchEngine() { }


}
