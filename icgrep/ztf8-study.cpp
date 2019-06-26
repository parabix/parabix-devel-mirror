/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>
#include <re/re_name.h>
#include <re/re_re.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <kernels/deletion.h>
#include <kernels/pdep_kernel.h>
#include <kernels/stream_select.h>
#include <kernels/stream_shift.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <grep/grep_engine.h>
#include <grep/grep_kernel.h>
#include <kernels/scanmatchgen.h>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <UCD/resolve_properties.h>
#include <UCD/ucd_compiler.hpp>
#include <string>
#include <toolchain/toolchain.h>
#include <re/re_name_resolve.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <fcntl.h>
#include <iostream>
#include <toolchain/cpudriver.h>

using namespace kernel;
using namespace llvm;
using namespace pablo;

static cl::OptionCategory ztf8Options("ztf8 Options", "ZTF-8 Compression/Decompression control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztf8Options));
static cl::opt<int> WordLen("length", cl::desc("Length of words."), cl::init(8));


class BitPositionsKernel : public MultiBlockKernel {
public:
    BitPositionsKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const markers, StreamSet * const positions);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
};

BitPositionsKernel::BitPositionsKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                       StreamSet * const markers, StreamSet * const positions)
: MultiBlockKernel(b, "bitPositions",
{Binding{"markers", markers}}, {Binding{"positions", positions, PopcountOf("markers")}}, {}, {}, {}) {}

void BitPositionsKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    IntegerType * const sizeTy = b->getSizeTy();
    // It is possible that we could scan with scan words of type other than size_t,
    // so we define a separate type for clarity.
    IntegerType * const scanWordTy = b->getSizeTy();
    Constant * sz_ZERO = Constant::getNullValue(sizeTy);
    Constant * sw_ZERO = Constant::getNullValue(scanWordTy);
    Constant * sz_ONE = ConstantInt::get(sizeTy, 1);
    Constant * sz_SW_BITS = ConstantInt::get(sizeTy, scanWordTy->getBitWidth());
    const unsigned scansPerStride = mStride / scanWordTy->getBitWidth();
    PointerType * const scanWordPointerType =  scanWordTy->getPointerTo();
    
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const forEachScanWord = b->CreateBasicBlock("forEachScanWord");
    BasicBlock * const forBitsInScanWord = b->CreateBasicBlock("forBitsInScanWord");
    BasicBlock * const scanWordDone = b->CreateBasicBlock("scanWordDone");
    BasicBlock * const scanWordsAllDone = b->CreateBasicBlock("scanWordsAllDone");
    Value * const markerStrmPos = b->getProcessedItemCount("markers");
    Value * positionCount = b->getProducedItemCount("positions");
    Value * numOfScanWords = b->CreateMul(numOfStrides, b->getSize(scansPerStride));
    Value * scanWordStreamPtr = b->getRawInputPointer("markers", sz_ZERO, markerStrmPos);
    scanWordStreamPtr = b->CreateBitCast(scanWordStreamPtr, scanWordPointerType);
    Value * positionStreamPtr = b->getRawOutputPointer("positions", sz_ZERO, positionCount);

    b->CreateBr(forEachScanWord);
    
    b->SetInsertPoint(forEachScanWord);
    PHINode * const scanWordIndex = b->CreatePHI(sizeTy, 2);
    scanWordIndex->addIncoming(sz_ZERO, entryBlock);
    PHINode * const scanWordPos = b->CreatePHI(sizeTy, 2);
    scanWordPos->addIncoming(markerStrmPos, entryBlock);
    PHINode * const positionIndex = b->CreatePHI(sizeTy, 2);
    positionIndex->addIncoming(sz_ZERO, entryBlock);
    
    Value * scanWord = b->CreateLoad(b->CreateGEP(scanWordStreamPtr, scanWordIndex), "markerBits");
    
    b->CreateCondBr(b->CreateICmpNE(scanWord, sw_ZERO), forBitsInScanWord, scanWordDone);
    
    b->SetInsertPoint(forBitsInScanWord);
    PHINode * const scanWordPhi = b->CreatePHI(scanWordTy, 2);
    scanWordPhi->addIncoming(scanWord, forEachScanWord);
    PHINode * const innerPositionIndex = b->CreatePHI(sizeTy, 2);
    innerPositionIndex->addIncoming(positionIndex, forEachScanWord);
    
    Value * markerPosn = b->CreateZExtOrTrunc(b->CreateCountForwardZeroes(scanWordPhi, true), sizeTy);
    markerPosn = b->CreateAdd(markerPosn, scanWordPos, "markerPosn");
    
    b->CreateStore(markerPosn, b->CreateGEP(positionStreamPtr, innerPositionIndex));
    Value * nextScanWord = b->CreateResetLowestBit(scanWordPhi);
    Value * nextInnerPositionIndex = b->CreateAdd(innerPositionIndex, sz_ONE);
    scanWordPhi->addIncoming(nextScanWord, forBitsInScanWord);
    innerPositionIndex->addIncoming(nextInnerPositionIndex, forBitsInScanWord);
    b->CreateCondBr(b->CreateICmpNE(nextScanWord, sw_ZERO), forBitsInScanWord, scanWordDone);
    
    b->SetInsertPoint(scanWordDone);
    PHINode * const positionAfterScanWord = b->CreatePHI(sizeTy, 2);
    positionAfterScanWord->addIncoming(positionIndex, forEachScanWord);
    positionAfterScanWord->addIncoming(nextInnerPositionIndex, forBitsInScanWord);
    
    Value * nextScanWordIdx = b->CreateAdd(scanWordIndex, sz_ONE);
    Value * nextScanWordPos = b->CreateAdd(scanWordPos, sz_SW_BITS);
    scanWordIndex->addIncoming(nextScanWordIdx, scanWordDone);
    scanWordPos->addIncoming(nextScanWordPos, scanWordDone);
    positionIndex->addIncoming(positionAfterScanWord, scanWordDone);
    
    b->CreateCondBr(b->CreateICmpNE(nextScanWordIdx, numOfScanWords), forEachScanWord, scanWordsAllDone);
    
    b->SetInsertPoint(scanWordsAllDone);
}


class WordMarkKernel : public PabloKernel {
public:
    WordMarkKernel(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * U8index, StreamSet * WordMarks);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

WordMarkKernel::WordMarkKernel(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * U8index, StreamSet * WordMarks)
: PabloKernel(kb, "RootMarks", {Binding{"source", BasisBits}, Binding{"U8index", U8index}}, {Binding{"WordMarks", WordMarks, FixedRate(), Add1()}}) { }

void WordMarkKernel::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("source"));
    UCD::UCDCompiler ucdCompiler(ccc);
    re::Name * word = re::makeName("word", re::Name::Type::UnicodeProperty);
    word = cast<re::Name>(re::resolveUnicodeNames(word));
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(word, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(word);
    if (f == nameMap.end()) llvm::report_fatal_error("Cannot find word property");
    PabloAST * wordChar = f->second;
    PabloAST * U8index = getInputStreamSet("U8index")[0];
    PabloAST * U8nonfinal = pb.createNot(U8index);
    
    PabloAST * nonWordChar = pb.createAnd(pb.createNot(wordChar), U8index);
    // Find the end of the encodeable substring root as well as the overall
    // end of the encodeable substring.   Care must be taken to ensure correct
    // behaviour at the beginning and end of file.   It is possible that the
    // root end is at position 0, if the first encodeable substring begins
    // with a non-word character.   It is also possible that the final position
    // of the root is at end of file if the final encodeable substring ends in
    // a word character.   The final word end is always at the EOF position.
    PabloAST * wordCharBehindOrStart = pb.createNot(pb.createAdvance(pb.createNot(wordChar), 1));
    PabloAST * afterWord = pb.createScanThru(wordCharBehindOrStart, U8nonfinal);
    PabloAST * rootEnd = pb.createAnd(afterWord, nonWordChar, "markRoot");
    PabloAST * afterNon = pb.createScanThru(pb.createAdvance(nonWordChar, 1), U8nonfinal);
    PabloAST * nextWordStart = pb.createAnd(afterNon, wordChar);
    PabloAST * endMark = pb.createOr(nextWordStart, pb.createAtEOF(pb.createOnes()), "markEnd");
    pb.createAssign(pb.createExtract(getOutputStreamVar("WordMarks"), pb.getInteger(0)), rootEnd);
    pb.createAssign(pb.createExtract(getOutputStreamVar("WordMarks"), pb.getInteger(1)), endMark);
}

class U8_Lookahead : public PabloKernel {
public:
    U8_Lookahead(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * WordMarks, StreamSet * wordEnds, StreamSet * wordBounds);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

U8_Lookahead::U8_Lookahead(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * WordMarks, StreamSet * RootMarks, StreamSet * WordEnds)
: PabloKernel(kb, "U8_Lookahead", {Binding{"source", BasisBits, FixedRate(1), ZeroExtended()}, Binding{"WordMarks", WordMarks, FixedRate(1), LookAhead(3)}}, {Binding{"RootMarks", RootMarks}, Binding{"WordEnds", WordEnds}}) { }

void U8_Lookahead::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> marks = getInputStreamSet("WordMarks");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("source"));
    PabloAST * ASCII = ccc.compileCC(re::makeCC(0x0, 0x7F));
    PabloAST * prefix2 = ccc.compileCC(re::makeCC(0xC2, 0xDF));
    PabloAST * prefix3 = ccc.compileCC(re::makeCC(0xE0, 0xEF));
    PabloAST * prefix4 = ccc.compileCC(re::makeCC(0xF0, 0xF4));
    PabloAST * rootEnds = pb.createOr(pb.createAnd(ASCII, marks[0]), pb.createAtEOF(marks[0]));
    rootEnds = pb.createOr(rootEnds, pb.createAnd(prefix2, pb.createLookahead(marks[0], 1)));
    rootEnds = pb.createOr(rootEnds, pb.createAnd(prefix3, pb.createLookahead(marks[0], 2)));
    rootEnds = pb.createOr(rootEnds, pb.createAnd(prefix4, pb.createLookahead(marks[0], 3)), "markedRootEnds");
    
    PabloAST * wordEnds = pb.createOr(pb.createAnd(ASCII, marks[1]), pb.createAtEOF(marks[1]));
    wordEnds = pb.createOr(wordEnds, pb.createAnd(prefix2, pb.createLookahead(marks[1], 1)));
    wordEnds = pb.createOr(wordEnds, pb.createAnd(prefix3, pb.createLookahead(marks[1], 2)));
    wordEnds = pb.createOr(wordEnds, pb.createAnd(prefix4, pb.createLookahead(marks[1], 3)), "markedWordEnds");
    pb.createAssign(pb.createExtract(getOutputStreamVar("RootMarks"), pb.getInteger(0)), rootEnds);
    pb.createAssign(pb.createExtract(getOutputStreamVar("WordEnds"), pb.getInteger(0)), wordEnds);
}

class WordAccumulator {
public:
    WordAccumulator() {}
    ~WordAccumulator() {}
    void accumulate_word(char * word_start, char * root_end, char * word_end);
    void dumpWords();
private:
    std::map<std::string, uint64_t> mWordBag;
    std::map<std::string, uint64_t> mExtensionBag;
};

void WordAccumulator::accumulate_word (char * word_start, char * root_end, char * word_end) {
    const auto bytes = word_end - word_start;
    std::string new_word(word_start, bytes);
    //llvm::errs() << "Found word: |" << new_word << "|\n";
    if (word_end != root_end) {
        std::string extension(root_end, word_end - root_end);
        //llvm::errs() << "Found extension: |" << extension << "|\n";
        mExtensionBag[extension]++;
    }
    mWordBag[new_word]++;
}

void WordAccumulator::dumpWords() {
    for (auto const& w : mWordBag) {
        llvm::errs() << "|" << w.first << "|: " << w.second << "\n";
    }
}

extern "C" void accumulate_word_wrapper(intptr_t accum_addr, char * word_start, char * root_end, char * word_end) {
    assert ("passed a null accumulator" && accum_addr);
    reinterpret_cast<WordAccumulator *>(accum_addr)->accumulate_word(word_start, root_end, word_end);
}

class ZTF8_Reporter : public SegmentOrientedKernel {
public:
    ZTF8_Reporter(const std::unique_ptr<kernel::KernelBuilder> & b,
                  StreamSet * ByteStream, StreamSet * const RootMarkPosns, StreamSet * constEndMarkPosns, Scalar * const callbackObject);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoSegmentMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};

ZTF8_Reporter::ZTF8_Reporter(const std::unique_ptr<kernel::KernelBuilder> & b,
                             StreamSet * ByteStream, StreamSet * const RootMarkPosns, StreamSet * const EndMarkPosns, Scalar * const callbackObject)
: SegmentOrientedKernel(b, "ztf8_Reporter",
                        // inputs
{Binding{"InputStream", ByteStream, GreedyRate(), Deferred()},
    Binding{"RootMarkPosns", RootMarkPosns, GreedyRate(1)},
    Binding{"EndMarkPosns", EndMarkPosns, GreedyRate(1)}},
                        // outputs
{},
                        // input scalars
{Binding{"accumulator_address", callbackObject}},
                        // output scalars
{},
                        // kernel state
{InternalScalar{b->getSizeTy(), "PendingWordStartPos"}}) {
    setStride(1);
    addAttribute(SideEffecting());
}


void ZTF8_Reporter::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {
    Module * const m = b->getModule();
    Constant * const sz_ONE = b->getSize(1);
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const processWords = b->CreateBasicBlock("processWords");
    BasicBlock * const coordinatesDone = b->CreateBasicBlock("coordinatesDone");

    Value * accumulator = b->getScalarField("accumulator_address");
    Value * wordsProcessed = b->getProcessedItemCount("RootMarkPosns");
    Value * wordsAvail = b->getAvailableItemCount("RootMarkPosns");
    Value * pendingWordStartPos = b->getScalarField("PendingWordStartPos");

    b->CreateCondBr(b->CreateICmpNE(wordsProcessed, wordsAvail), processWords, coordinatesDone);

    b->SetInsertPoint(processWords);
    PHINode * phiWordStart = b->CreatePHI(b->getSizeTy(), 2, "phiWordStart");
    phiWordStart->addIncoming(pendingWordStartPos, entryBlock);
    PHINode * phiWordNum = b->CreatePHI(b->getSizeTy(), 2, "wordNum");
    phiWordNum->addIncoming(wordsProcessed, entryBlock);
    Value * rootEndPosn = b->CreateLoad(b->getRawInputPointer("RootMarkPosns", b->getInt32(0), phiWordNum), "rootEndPosn");
    Value * wordEndPosn = b->CreateLoad(b->getRawInputPointer("EndMarkPosns", b->getInt32(0), phiWordNum), "wordEndPosn");
    Value * wordStartPtr = b->getRawInputPointer("InputStream", phiWordStart);
    Value * rootEndPtr = b->getRawInputPointer("InputStream", rootEndPosn);
    Value * wordEndPtr = b->getRawInputPointer("InputStream", wordEndPosn);
    Function * const dispatcher = m->getFunction("accumulate_word_wrapper"); assert (dispatcher);
    b->CreateCall(dispatcher, {accumulator, wordStartPtr, rootEndPtr, wordEndPtr});
    Value * nextWordNum = b->CreateAdd(phiWordNum, sz_ONE);
    Value * haveMoreWords = b->CreateICmpNE(nextWordNum, wordsAvail);
    phiWordStart->addIncoming(wordEndPosn, b->GetInsertBlock());
    phiWordNum->addIncoming(nextWordNum, b->GetInsertBlock());
    b->CreateCondBr(haveMoreWords, processWords, coordinatesDone);
    b->SetInsertPoint(coordinatesDone);
}

typedef void (*ztf8FunctionType)(uint32_t fd, WordAccumulator *);

ztf8FunctionType generatePipeline(CPUDriver & pxDriver) {
    
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"},
                                    Binding{b->getIntAddrTy(), "callbackObject"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);
    
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    
    StreamSet * const U8index = P->CreateStreamSet(1);
    P->CreateKernelCall<UTF8_index>(BasisBits, U8index);
    
    StreamSet * WordMarks = P->CreateStreamSet(2);
    P->CreateKernelCall<WordMarkKernel>(BasisBits, U8index, WordMarks);
    
    StreamSet * RootMarks = P->CreateStreamSet(1);
    StreamSet * EndMarks = P->CreateStreamSet(1);
    P->CreateKernelCall<U8_Lookahead>(BasisBits, WordMarks, RootMarks, EndMarks);

    StreamSet * RootMarkPosns = P->CreateStreamSet(1, sizeof(size_t) * 8);
    StreamSet * EndMarkPosns = P->CreateStreamSet(1, sizeof(size_t) * 8);
    
    P->CreateKernelCall<BitPositionsKernel>(RootMarks, RootMarkPosns);
    P->CreateKernelCall<BitPositionsKernel>(EndMarks, EndMarkPosns);

    Scalar * const callbackObject = P->getInputScalar("callbackObject");
    
    Kernel * reporterK = P->CreateKernelCall<ZTF8_Reporter>(ByteStream, RootMarkPosns, EndMarkPosns, callbackObject);
    pxDriver.LinkFunction(reporterK, "accumulate_word_wrapper", accumulate_word_wrapper);

    return reinterpret_cast<ztf8FunctionType>(P->compile());
}

int main(int argc, char *argv[]) {

    codegen::ParseCommandLineOptions(argc, argv, {&ztf8Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver driver("ztf8");
    WordAccumulator accum;
    ztf8FunctionType fn = generatePipeline(driver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        fn(fd, &accum);
        close(fd);
    }
    accum.dumpWords();
    return 0;
}
