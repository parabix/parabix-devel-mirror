/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <set>
#include "grep_engine.h"
#include <llvm/IR/Module.h>
#include <boost/filesystem.hpp>
#include <UCD/resolve_properties.h>
#include <kernels/charclasses.h>
#include <kernels/cc_kernel.h>
#include <kernels/grep_kernel.h>
#include <kernels/UCD_property_kernel.h>
#include <kernels/grapheme_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/until_n.h>
#include <kernels/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <cc/alphabet.h>
#include <re/re_cc.h>
#include <re/re_alt.h>
#include <re/re_end.h>
#include <re/re_name.h>
#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/to_utf8.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <re/re_analysis.h>
#include <re/re_name_resolve.h>
#include <re/re_name_gather.h>
#include <re/collect_ccs.h>
#include <re/replaceCC.h>
#include <re/re_multiplex.h>
#include <re/grapheme_clusters.h>
#include <re/re_utility.h>
#include <re/printer_re.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <iostream>
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/file_select.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Casting.h>
#include <sched.h>

using namespace parabix;
using namespace llvm;
using namespace cc;
using namespace kernel;

static cl::opt<int> Threads("t", cl::desc("Total number of threads."), cl::init(2));
static cl::opt<bool> PabloTransposition("enable-pablo-s2p", cl::desc("Enable experimental pablo transposition."));
static cl::opt<bool> CC_Multiplexing("CC-multiplexing", cl::desc("Enable CC multiplexing."), cl::init(false));
static cl::opt<bool> PropertyKernels("enable-property-kernels", cl::desc("Enable Unicode property kernels."), cl::init(false));
static cl::opt<bool> MultithreadedSimpleRE("enable-simple-RE-kernels", cl::desc("Enable individual CC kernels for simple REs."), cl::init(false));
const unsigned DefaultByteCClimit = 6;

static cl::opt<unsigned> ByteCClimit("byte-CC-limit", cl::desc("Max number of CCs for byte CC pipeline."), cl::init(DefaultByteCClimit));


namespace grep {
    
extern "C" void signal_dispatcher(intptr_t callback_object_addr, unsigned signal) {
    reinterpret_cast<GrepCallBackObject *>(callback_object_addr)->handle_signal(signal);
}
    
void GrepCallBackObject::handle_signal(unsigned s) {
    if (static_cast<GrepSignal>(s) == GrepSignal::BinaryFile) {
        mBinaryFile = true;
    } else {
        llvm::report_fatal_error("Unknown GrepSignal");
    }
}

extern "C" void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end) {
    reinterpret_cast<MatchAccumulator *>(accum_addr)->accumulate_match(lineNum, line_start, line_end);
}

extern "C" void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end) {
    reinterpret_cast<MatchAccumulator *>(accum_addr)->finalize_match(buffer_end);
}
    

inline static size_t ceil_log2(const size_t v) {
    assert ("log2(0) is undefined!" && v != 0);
    assert ("sizeof(size_t) == sizeof(long)" && sizeof(size_t) == sizeof(long));
    return (sizeof(size_t) * CHAR_BIT) - __builtin_clzl(v - 1UL);
}

void SearchableBuffer::addSearchCandidate(const char * C_string_ptr) {
    size_t length = strlen(C_string_ptr)+1;
    if (mSpace_used + length >= mAllocated_capacity) {
        size_t new_capacity = size_t{1} << (ceil_log2(mSpace_used + length + 1));
        AlignedAllocator<char, BUFFER_ALIGNMENT> alloc;
        char * new_buffer = mAllocator.allocate(new_capacity, 0);
        memcpy(new_buffer, mBuffer_base, mSpace_used);
        memset(&new_buffer[mSpace_used], 0, new_capacity-mSpace_used);
        if (mBuffer_base != mInitial_buffer) {
            alloc.deallocate(mBuffer_base, 0);
        }
        mBuffer_base = new_buffer;
        mAllocated_capacity = new_capacity;
    }
    memcpy((void * ) &mBuffer_base[mSpace_used], C_string_ptr, length);
    mSpace_used += length;
    assert("Search candidate not null terminated" && (mBuffer_base[mSpace_used] == '\0'));
    mEntries++;
}

SearchableBuffer::SearchableBuffer() :
    mAllocated_capacity(INITIAL_CAPACITY),
    mSpace_used(0),
    mEntries(0),
    mBuffer_base(mInitial_buffer) {
    memset(mBuffer_base, 0, INITIAL_CAPACITY);
}

SearchableBuffer::~SearchableBuffer() {
    if (mBuffer_base != mInitial_buffer) {
        mAllocator.deallocate(mBuffer_base, 0);
    }
}



// Grep Engine construction and initialization.

GrepEngine::GrepEngine() :
    mSuppressFileMessages(false),
    mBinaryFilesMode(argv::Text),
    mPreferMMap(true),
    mShowFileNames(false),
    mStdinLabel("(stdin)"),
    mShowLineNumbers(false),
    mInitialTab(false),
    mCaseInsensitive(false),
    mInvertMatches(false),
    mMaxCount(0),
    mGrepStdIn(false),
    mGrepDriver(make_unique<ParabixDriver>("engine")),
    mNextFileToGrep(0),
    mNextFileToPrint(0),
    grepMatchFound(false),
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mMoveMatchesToEOL(true),
    mEngineThread(pthread_self()) {}

QuietModeEngine::QuietModeEngine() : GrepEngine() {
    mEngineKind = EngineKind::QuietMode;
    mMoveMatchesToEOL = false;
    mMaxCount = 1;
}

MatchOnlyEngine::MatchOnlyEngine(bool showFilesWithMatch, bool useNullSeparators) :
    GrepEngine(), mRequiredCount(showFilesWithMatch) {
    mEngineKind = EngineKind::MatchOnly;
    mFileSuffix = useNullSeparators ? std::string("\0", 1) : "\n";
    mMoveMatchesToEOL = false;
    mMaxCount = 1;
    mShowFileNames = true;
}

CountOnlyEngine::CountOnlyEngine() : GrepEngine() {
    mEngineKind = EngineKind::CountOnly;
    mFileSuffix = ":";
}

EmitMatchesEngine::EmitMatchesEngine() : GrepEngine() {
    mEngineKind = EngineKind::EmitMatches;
    mFileSuffix = mInitialTab ? "\t:" : ":";
}

    
void GrepEngine::setRecordBreak(GrepRecordBreakKind b) {
    mGrepRecordBreak = b;
}

    

    
void GrepEngine::initFileResult(std::vector<boost::filesystem::path> & paths) {
    const unsigned n = paths.size();
    mResultStrs.resize(n);
    mFileStatus.resize(n, FileStatus::Pending);
    inputPaths = paths;
}

void GrepEngine::initREs(std::vector<re::RE *> & REs) {
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        mBreakCC = re::makeCC(re::makeCC(0x0A, 0x0D), re::makeCC(re::makeCC(0x85), re::makeCC(0x2028, 0x2029)));
    } else if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        mBreakCC = re::makeByte(0);  // Null
    } else {
        mBreakCC = re::makeByte(0x0A); // LF
    }
    re::RE * anchorRE = mBreakCC;
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        re::Name * anchorName = re::makeName("UTF8_LB", re::Name::Type::Unicode);
        anchorName->setDefinition(re::makeUnicodeBreak());
        anchorRE = anchorName;
    }
    
    mREs = REs;
    bool allAnchored = true;
    for(unsigned i = 0; i < mREs.size(); ++i) {
        if (!hasEndAnchor(mREs[i])) allAnchored = false;
        mREs[i] = resolveModesAndExternalSymbols(mREs[i], mCaseInsensitive);
        mREs[i] = re::exclude_CC(mREs[i], mBreakCC);
        mREs[i] = resolveAnchors(mREs[i], anchorRE);
        re::gatherUnicodeProperties(mREs[i], mUnicodeProperties);
        mREs[i] = regular_expression_passes(mREs[i]);
    }
    if (allAnchored && (mGrepRecordBreak != GrepRecordBreakKind::Unicode)) mMoveMatchesToEOL = false;

}


    
// Code Generation
//
// All engines share a common pipeline to compute a stream of Matches from a given input Bytestream.

unsigned LLVM_READNONE calculateMaxCountRate(const std::unique_ptr<kernel::KernelBuilder> & b) {
    const unsigned packSize = b->getSizeTy()->getBitWidth();
    return (packSize * packSize) / b->getBitBlockWidth();
}
    
std::pair<StreamSetBuffer *, StreamSetBuffer *> GrepEngine::grepPipeline(StreamSetBuffer * SourceStream, Value * callback_object_addr) {
    auto & idb = mGrepDriver->getBuilder();
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    // TODO: until we automate stream buffer sizing, use this calculation to determine how large our matches buffer needs to be.
    const unsigned baseBufferSize = segmentSize * (mMaxCount > 0 ? (std::max(bufferSegments, calculateMaxCountRate(idb))) : bufferSegments);
    const unsigned encodingBits = 8;
    
    
    //  Regular Expression Processing and Analysis Phase
    const auto nREs = mREs.size();
    bool hasGCB[nREs];
    bool anyGCB = false;

    for(unsigned i = 0; i < nREs; ++i) {
        hasGCB[i] = hasGraphemeClusterBoundary(mREs[i]);
        anyGCB |= hasGCB[i];
    }
    StreamSetBuffer * ByteStream = nullptr;
    if (mBinaryFilesMode == argv::Text) {
        ByteStream = SourceStream;
    } else if (mBinaryFilesMode == argv::WithoutMatch) {
        ByteStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 8), baseBufferSize);
        kernel::Kernel * binaryCheckK = mGrepDriver->addKernelInstance<kernel::AbortOnNull>(idb);
        binaryCheckK->setInitialArguments({callback_object_addr});
        mGrepDriver->makeKernelCall(binaryCheckK, {SourceStream}, {ByteStream});
        mGrepDriver->LinkFunction(*binaryCheckK, "signal_dispatcher", &signal_dispatcher);
    } else {
        llvm::report_fatal_error("Binary mode not supported.");
    }
    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
    std::vector<StreamSetBuffer *> MatchResultsBufs(nREs);
    
    re::RE * prefixRE;
    re::RE * suffixRE;
    // For simple regular expressions with a small number of characters, we
    // can bypass transposition and use the Direct CC compiler.
    bool isSimple = (nREs == 1) && (mGrepRecordBreak != GrepRecordBreakKind::Unicode) && (!anyGCB);
    if (isSimple) {
        mREs[0] = toUTF8(mREs[0]);
    }
    if (isSimple && byteTestsWithinLimit(mREs[0], ByteCClimit)) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {ByteStream};
        if (MultithreadedSimpleRE && hasTriCCwithinLimit(mREs[0], ByteCClimit, prefixRE, suffixRE)) {
            auto CCs = re::collectCCs(prefixRE, &cc::Byte);
            for (auto cc : CCs) {
                auto ccName = makeName(cc);
                mREs[0] = re::replaceCC(mREs[0], cc, ccName);
                std::string ccNameStr = ccName->getFullName();
                StreamSetBuffer * ccStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, ccNameStr, std::vector<re::CC *>{cc});
                mGrepDriver->makeKernelCall(ccK, {ByteStream}, {ccStream});
                externalStreamNames.push_back(ccNameStr);
                icgrepInputSets.push_back(ccStream);
            }
        }
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ByteGrepKernel>(idb, mREs[0], externalStreamNames);
        mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
        MatchResultsBufs[0] = MatchResults;
        kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, "breakCC", std::vector<re::CC *>{mBreakCC});
        mGrepDriver->makeKernelCall(breakK, {ByteStream}, {LineBreakStream});
    } else if (isSimple && hasTriCCwithinLimit(mREs[0], ByteCClimit, prefixRE, suffixRE)) {
        std::vector<std::string> externalStreamNames;
        std::vector<StreamSetBuffer *> icgrepInputSets = {ByteStream};
        if (MultithreadedSimpleRE) {
            auto CCs = re::collectCCs(prefixRE, &cc::Byte);
            for (auto cc : CCs) {
                auto ccName = makeName(cc);
                mREs[0] = re::replaceCC(mREs[0], cc, ccName);
                std::string ccNameStr = ccName->getFullName();
                StreamSetBuffer * ccStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, ccNameStr, std::vector<re::CC *>{cc});
                mGrepDriver->makeKernelCall(ccK, {ByteStream}, {ccStream});
                externalStreamNames.push_back(ccNameStr);
                icgrepInputSets.push_back(ccStream);
            }
        }
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ByteBitGrepKernel>(idb, prefixRE, suffixRE, externalStreamNames);
        mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
        MatchResultsBufs[0] = MatchResults;
        kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, "breakCC", std::vector<re::CC *>{mBreakCC});
        mGrepDriver->makeKernelCall(breakK, {ByteStream}, {LineBreakStream});
    } else {
        
        StreamSetBuffer * BasisBits = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(encodingBits, 1), baseBufferSize);
        kernel::Kernel * s2pk = nullptr;
        if (PabloTransposition) {
            s2pk = mGrepDriver->addKernelInstance<kernel::S2P_PabloKernel>(idb);
        }
        else {
            s2pk = mGrepDriver->addKernelInstance<kernel::S2PKernel>(idb);
        }
        mGrepDriver->makeKernelCall(s2pk, {ByteStream}, {BasisBits});

        StreamSetBuffer * RequiredStreams = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        StreamSetBuffer * UnicodeLB = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);

        StreamSetBuffer * LineFeedStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * linefeedK = mGrepDriver->addKernelInstance<kernel::LineFeedKernelBuilder>(idb, Binding{idb->getStreamSetTy(8), "basis", FixedRate(), Principal()});
        mGrepDriver->makeKernelCall(linefeedK, {BasisBits}, {LineFeedStream});
        
        kernel::Kernel * requiredStreamsK = mGrepDriver->addKernelInstance<kernel::RequiredStreams_UTF8>(idb);
        mGrepDriver->makeKernelCall(requiredStreamsK, {BasisBits, LineFeedStream}, {RequiredStreams, UnicodeLB});

        if (mGrepRecordBreak == GrepRecordBreakKind::LF) {
            LineBreakStream = LineFeedStream;
        } else if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
            kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::ParabixCharacterClassKernelBuilder>(idb, "Null", std::vector<re::CC *>{mBreakCC}, 8);
            mGrepDriver->makeKernelCall(breakK, {BasisBits}, {LineBreakStream});
        } else {
            LineBreakStream = UnicodeLB;
        }
        
        std::map<std::string, StreamSetBuffer *> propertyStream;
        if (PropertyKernels) {
            for (auto p : mUnicodeProperties) {
                auto name = p->getFullName();
                StreamSetBuffer * s = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                propertyStream.emplace(std::make_pair(name, s));
                kernel::Kernel * propertyK = mGrepDriver->addKernelInstance<kernel::UnicodePropertyKernelBuilder>(idb, p);
                mGrepDriver->makeKernelCall(propertyK, {BasisBits}, {s});
            }
        }
        StreamSetBuffer * GCB_stream = nullptr;
        if (anyGCB) {
            GCB_stream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
            kernel::Kernel * gcbK = mGrepDriver->addKernelInstance<kernel::GraphemeClusterBreakKernel>(idb);
            mGrepDriver->makeKernelCall(gcbK, {BasisBits, RequiredStreams}, {GCB_stream});
        }

        for(unsigned i = 0; i < nREs; ++i) {
            std::vector<std::string> externalStreamNames;
            std::vector<StreamSetBuffer *> icgrepInputSets = {BasisBits};
            if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
                externalStreamNames.push_back("UTF8_LB");
                icgrepInputSets.push_back(LineBreakStream);
                externalStreamNames.push_back("UTF8_nonfinal");
                icgrepInputSets.push_back(RequiredStreams);
            }
            std::set<re::Name *> UnicodeProperties;
            if (PropertyKernels) {
                re::gatherUnicodeProperties(mREs[i], UnicodeProperties);
                for (auto p : UnicodeProperties) {
                    auto name = p->getFullName();
                    auto f = propertyStream.find(name);
                    if (f == propertyStream.end()) report_fatal_error(name + " not found\n");
                    externalStreamNames.push_back(name);
                    icgrepInputSets.push_back(f->second);
                }
            }
            if (hasGCB[i]) {
                externalStreamNames.push_back("\\b{g}");
                icgrepInputSets.push_back(GCB_stream);
            }
            if (CC_Multiplexing) {
                const auto UnicodeSets = re::collectCCs(mREs[i], &cc::Unicode, std::set<re::Name *>({re::makeZeroWidth("\\b{g}")}));
                StreamSetBuffer * const MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                if (UnicodeSets.size() <= 1) {
                    kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames);
                    mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
                    MatchResultsBufs[i] = MatchResults;
                } else {
                    mpx = make_unique<MultiplexedAlphabet>("mpx", UnicodeSets);
                    mREs[i] = transformCCs(mpx.get(), mREs[i]);
                    std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
                    auto numOfCharacterClasses = mpx_basis.size();
                    StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), baseBufferSize);
                    kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis));
                    mGrepDriver->makeKernelCall(ccK, {BasisBits}, {CharClasses});
    //                kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis), true);
    //                mGrepDriver->makeKernelCall(ccK, {ByteStream}, {CharClasses});
                    kernel::ICGrepKernel * icgrepK = (kernel::ICGrepKernel*)mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames, std::vector<cc::Alphabet *>{mpx.get()}, cc::BitNumbering::LittleEndian);
                    // Multiplexing Grep Kernel is not Cachable, since for now it use string representation of RE AST as cache key,
                    // whileit is possible that two multiplexed REs with the same name "mpx_1" have different alphabets
                    icgrepK->setCachable(false);
                    icgrepInputSets.push_back(CharClasses);
                    mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
                    MatchResultsBufs[i] = MatchResults;
                }
            } else {
                StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
                kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, mREs[i], externalStreamNames);
                mGrepDriver->makeKernelCall(icgrepK, icgrepInputSets, {MatchResults});
                MatchResultsBufs[i] = MatchResults;
            }
        }
    }

    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (mREs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance<kernel::StreamsMerge>(idb, 1, mREs.size());
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;
    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    if (mInvertMatches) {
        kernel::Kernel * invertK = mGrepDriver->addKernelInstance<kernel::InvertMatchesKernel>(idb);
        StreamSetBuffer * OriginalMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(invertK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    if (mMaxCount > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(mMaxCount)});
        StreamSetBuffer * const AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), baseBufferSize);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }

    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);
}

// The QuietMode, MatchOnly and CountOnly engines share a common code generation main function,
// which returns a count of the matches found (possibly subject to a MaxCount).
//

void GrepEngine::grepCodeGen() {
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const unsigned encodingBits = 8;

    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getInt64Ty(), idb->getInt8Ty(), idb->getInt32Ty(), idb->getIntAddrTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const useMMap = &*(args++);
    useMMap->setName("useMMap");
    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * call_back_object = &*(args++);
    call_back_object->setName("call_back_object");

    StreamSetBuffer * const ByteStream = mGrepDriver->addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, encodingBits));
    kernel::Kernel * const sourceK = mGrepDriver->addKernelInstance<kernel::FDSourceKernel>(idb);
    sourceK->setInitialArguments({useMMap, fileDescriptor});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(ByteStream, call_back_object);

    kernel::Kernel * matchCountK = mGrepDriver->addKernelInstance<kernel::PopcountKernel>(idb);
    mGrepDriver->makeKernelCall(matchCountK, {Matches}, {});
    mGrepDriver->generatePipelineIR();
    idb->setKernel(matchCountK);
    Value * matchedLineCount = idb->getAccumulator("countResult");
    matchedLineCount = idb->CreateZExt(matchedLineCount, idb->getInt64Ty());
    mGrepDriver->deallocateBuffers();
    idb->CreateRet(matchedLineCount);
    
    mGrepDriver->finalizeObject();
}

//
//  Default Report Match:  lines are emitted with whatever line terminators are found in the
//  input.  However, if the final line is not terminated, a new line is appended.
//
void EmitMatch::accumulate_match (const size_t lineNum, char * line_start, char * line_end) {
    mResultStr << mLinePrefix;
    if (mShowLineNumbers) {
        // Internally line numbers are counted from 0.  For display, adjust
        // the line number so that lines are numbered from 1.
        if (mInitialTab) {
            mResultStr << lineNum+1 << "\t:";
        }
        else {
            mResultStr << lineNum+1 << ":";
        }
    }
    size_t bytes = line_end - line_start + 1;
    mResultStr.write(line_start, bytes);
    mLineCount++;
    unsigned last_byte = *line_end;
    mTerminated = (last_byte >= 0x0A) && (last_byte <= 0x0D);
    if (LLVM_UNLIKELY(!mTerminated)) {
        if (last_byte == 0x85) {  //  Possible NEL terminator.
            mTerminated = (bytes >= 2) && (static_cast<unsigned>(line_end[-1]) == 0xC2);
        }
        else {
            // Possible LS or PS terminators.
            mTerminated = (bytes >= 3) && (static_cast<unsigned>(line_end[-2]) == 0xE2)
                                       && (static_cast<unsigned>(line_end[-1]) == 0x80)
                                       && ((last_byte == 0xA8) || (last_byte == 0xA9));
        }
    }
}

void EmitMatch::finalize_match(char * buffer_end) {
    if (!mTerminated) mResultStr << "\n";
}

void EmitMatchesEngine::grepCodeGen() {
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const unsigned encodingBits = 8;

    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getInt64Ty(), idb->getInt8Ty(), idb->getInt32Ty(), idb->getIntAddrTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const useMMap = &*(args++);
    useMMap->setName("useMMap");
    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * match_accumulator = &*(args++);
    match_accumulator->setName("match_accumulator");

    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, encodingBits));
    kernel::Kernel * sourceK = mGrepDriver->addKernelInstance<kernel::FDSourceKernel>(idb);
    sourceK->setInitialArguments({useMMap, fileDescriptor});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(ByteStream, match_accumulator);

    kernel::Kernel * scanMatchK = mGrepDriver->addKernelInstance<kernel::ScanMatchKernel>(idb);
    scanMatchK->setInitialArguments({match_accumulator});
    mGrepDriver->makeKernelCall(scanMatchK, {Matches, LineBreakStream, ByteStream}, {});
    mGrepDriver->LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    mGrepDriver->LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);

    mGrepDriver->generatePipelineIR();
    mGrepDriver->deallocateBuffers();
    idb->CreateRet(idb->getInt64(0));
    mGrepDriver->finalizeObject();
}


//
//  The doGrep methods apply a GrepEngine to a single file, processing the results
//  differently based on the engine type.

bool canMMap(const std::string & fileName) {
    if (fileName == "-") return false;
    namespace fs = boost::filesystem;
    fs::path p(fileName);
    boost::system::error_code errc;
    fs::file_status s = fs::status(p, errc);
    return !errc && is_regular_file(s);
}


uint64_t GrepEngine::doGrep(const std::string & fileName, std::ostringstream & strm) {
    typedef uint64_t (*GrepFunctionType)(bool useMMap, int32_t fileDescriptor, intptr_t callback_addr);
    bool useMMap = mPreferMMap && canMMap(fileName);

    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());

    int32_t fileDescriptor = openFile(fileName, strm);
    if (fileDescriptor == -1) return 0;
    GrepCallBackObject handler;
    uint64_t grepResult = f(useMMap, fileDescriptor, reinterpret_cast<intptr_t>(&handler));
    close(fileDescriptor);
    if (handler.binaryFileSignalled()) {
        return 0;
    }
    else {
        showResult(grepResult, fileName, strm);
        return grepResult;
    }
}

std::string GrepEngine::linePrefix(std::string fileName) {
    if (!mShowFileNames) return "";
    if (fileName == "-") {
        return mStdinLabel + mFileSuffix;
    }
    else {
        return fileName + mFileSuffix;
    }
}

// Default: do not show anything
void GrepEngine::showResult(uint64_t grepResult, const std::string & fileName, std::ostringstream & strm) {
}
    
void CountOnlyEngine::showResult(uint64_t grepResult, const std::string & fileName, std::ostringstream & strm) {
    if (mShowFileNames) strm << linePrefix(fileName);
    strm << grepResult << "\n";
}
    
void MatchOnlyEngine::showResult(uint64_t grepResult, const std::string & fileName, std::ostringstream & strm) {
    if (grepResult == mRequiredCount) {
       strm << linePrefix(fileName);
    }
}

uint64_t EmitMatchesEngine::doGrep(const std::string & fileName, std::ostringstream & strm) {
    typedef uint64_t (*GrepFunctionType)(bool useMMap, int32_t fileDescriptor, intptr_t accum_addr);
    bool useMMap = mPreferMMap && canMMap(fileName);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());
    int32_t fileDescriptor = openFile(fileName, strm);
    if (fileDescriptor == -1) return 0;
    EmitMatch accum(linePrefix(fileName), mShowLineNumbers, mInitialTab, strm);
    f(useMMap, fileDescriptor, reinterpret_cast<intptr_t>(&accum));
    close(fileDescriptor);
    if (accum.binaryFileSignalled()) {
        accum.mResultStr.clear();
    }
    if (accum.mLineCount > 0) grepMatchFound = true;
    return accum.mLineCount;
}

// Open a file and return its file desciptor.
int32_t GrepEngine::openFile(const std::string & fileName, std::ostringstream & msgstrm) {
    if (fileName == "-") {
        return STDIN_FILENO;
    }
    else {
        struct stat sb;
        int32_t fileDescriptor = open(fileName.c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fileDescriptor == -1)) {
            if (!mSuppressFileMessages) {
                if (errno == EACCES) {
                    msgstrm << "icgrep: " << fileName << ": Permission denied.\n";
                }
                else if (errno == ENOENT) {
                    msgstrm << "icgrep: " << fileName << ": No such file.\n";
                }
                else {
                    msgstrm << "icgrep: " << fileName << ": Failed.\n";
                }
            }
            return fileDescriptor;
        }
        if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
            if (!mSuppressFileMessages) {
                msgstrm << "icgrep: " << fileName << ": Is a directory.\n";
            }
            close(fileDescriptor);
            return -1;
        }
        return fileDescriptor;
    }
}

// The process of searching a group of files may use a sequential or a task
// parallel approach.

void * DoGrepThreadFunction(void *args) {
    return reinterpret_cast<GrepEngine *>(args)->DoGrepThreadMethod();
}

bool GrepEngine::searchAllFiles() {
    const unsigned numOfThreads = std::min(static_cast<unsigned>(Threads), static_cast<unsigned>(inputPaths.size()));
    std::vector<pthread_t> threads(numOfThreads);

    for(unsigned long i = 1; i < numOfThreads; ++i) {
        const int rc = pthread_create(&threads[i], nullptr, DoGrepThreadFunction, (void *)this);
        if (rc) {
            llvm::report_fatal_error("Failed to create thread: code " + std::to_string(rc));
        }
    }
    // Main thread also does the work;
    DoGrepThreadMethod();
    for(unsigned i = 1; i < numOfThreads; ++i) {
        void * status = nullptr;
        const int rc = pthread_join(threads[i], &status);
        if (rc) {
            llvm::report_fatal_error("Failed to join thread: code " + std::to_string(rc));
        }
    }
    return grepMatchFound;
}


// DoGrep thread function.
void * GrepEngine::DoGrepThreadMethod() {

    unsigned fileIdx = mNextFileToGrep++;
    while (fileIdx < inputPaths.size()) {
        if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
            errs() << "Tracing " << inputPaths[fileIdx].string() << "\n";
        }
        const auto grepResult = doGrep(inputPaths[fileIdx].string(), mResultStrs[fileIdx]);
        mFileStatus[fileIdx] = FileStatus::GrepComplete;
        if (grepResult > 0) {
            grepMatchFound = true;
        }
        if ((mEngineKind == EngineKind::QuietMode) && grepMatchFound) {
            if (pthread_self() != mEngineThread) {
                pthread_exit(nullptr);
            }
            return nullptr;
        }
        fileIdx = mNextFileToGrep++;
    }

    unsigned printIdx = mNextFileToPrint++;
    while (printIdx < inputPaths.size()) {
        const bool readyToPrint = ((printIdx == 0) || (mFileStatus[printIdx - 1] == FileStatus::PrintComplete)) && (mFileStatus[printIdx] == FileStatus::GrepComplete);
        if (readyToPrint) {
            const auto output = mResultStrs[printIdx].str();
            if (!output.empty()) {
                llvm::outs() << output;
            }
            mFileStatus[printIdx] = FileStatus::PrintComplete;
            printIdx = mNextFileToPrint++;
        } else {
            mGrepDriver->performIncrementalCacheCleanupStep();
        }
        sched_yield();
    }

    if (pthread_self() != mEngineThread) {
        pthread_exit(nullptr);
    } else {
        // Always perform one final cache cleanup step.
        mGrepDriver->performIncrementalCacheCleanupStep();
        if (mGrepStdIn) {
            std::ostringstream s;
            const auto grepResult = doGrep("-", s);
            llvm::outs() << s.str();
            if (grepResult) grepMatchFound = true;
        }
    }
    return nullptr;
}

    
    
InternalSearchEngine::InternalSearchEngine() :
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mCaseInsensitive(false),
    mGrepDriver(make_unique<ParabixDriver>("InternalEngine")) {}
    
void InternalSearchEngine::grepCodeGen(re::RE * matchingRE, re::RE * excludedRE, MatchAccumulator * accum) {
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();
    
    mSaveSegmentPipelineParallel = codegen::SegmentPipelineParallel;
    codegen::SegmentPipelineParallel = false;
    const unsigned segmentSize = codegen::BufferSegments * codegen::SegmentSize * codegen::ThreadNum;
    
    re::CC * breakCC = nullptr;
    if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        breakCC = re::makeByte(0x0);
    } else {// if (mGrepRecordBreak == GrepRecordBreakKind::LF)
        breakCC = re::makeByte(0x0A);
    }
    bool excludeNothing = (excludedRE == nullptr) || (isa<re::Alt>(excludedRE) && cast<re::Alt>(excludedRE)->empty());
    bool matchAllLines = (matchingRE == nullptr) || isa<re::End>(matchingRE);
    if (!matchAllLines) {
        matchingRE = resolveCaseInsensitiveMode(matchingRE, mCaseInsensitive);
        matchingRE = regular_expression_passes(matchingRE);
        matchingRE = re::exclude_CC(matchingRE, breakCC);
        matchingRE = resolveAnchors(matchingRE, breakCC);
        matchingRE = toUTF8(matchingRE);
    }
    if (!excludeNothing) {
        excludedRE = resolveCaseInsensitiveMode(excludedRE, mCaseInsensitive);
        excludedRE = regular_expression_passes(excludedRE);
        excludedRE = re::exclude_CC(excludedRE, breakCC);
        excludedRE = resolveAnchors(excludedRE, breakCC);
        excludedRE = toUTF8(excludedRE);
    }
    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getVoidTy(), idb->getInt8PtrTy(), idb->getSizeTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    auto args = mainFunc->arg_begin();
    Value * const buffer = &*(args++);
    buffer->setName("buffer");
    Value * length = &*(args++);
    length->setName("length");
    
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK = mGrepDriver->addKernelInstance<kernel::MemorySourceKernel>(idb);
    sourceK->setInitialArguments({buffer, length});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});
    StreamSetBuffer * RecordBreakStream = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
    std::string RBname = (mGrepRecordBreak == GrepRecordBreakKind::Null) ? "Null" : "LF";

    
    StreamSetBuffer * BasisBits = nullptr;
    
    if (matchAllLines && excludeNothing) {
        kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, RBname, std::vector<re::CC *>{breakCC});
        mGrepDriver->makeKernelCall(breakK, {ByteStream}, {RecordBreakStream});
    } else {
        BasisBits = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(8, 1), segmentSize);
        kernel::Kernel * s2pk = mGrepDriver->addKernelInstance<kernel::S2PKernel>(idb);
        mGrepDriver->makeKernelCall(s2pk, {ByteStream}, {BasisBits});
        
        kernel::Kernel * breakK = mGrepDriver->addKernelInstance<kernel::ParabixCharacterClassKernelBuilder>(idb, RBname, std::vector<re::CC *>{breakCC}, 8);
        mGrepDriver->makeKernelCall(breakK, {BasisBits}, {RecordBreakStream});
    }
    
    std::vector<std::string> externalStreamNames;
    StreamSetBuffer * MatchingRecords = nullptr;
    if (matchAllLines) {
        MatchingRecords = RecordBreakStream;
    } else {
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
        kernel::Kernel * includeK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, matchingRE, externalStreamNames);
        mGrepDriver->makeKernelCall(includeK, {BasisBits}, {MatchResults});
        MatchingRecords = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        mGrepDriver->makeKernelCall(matchedLinesK, {MatchResults, RecordBreakStream}, {MatchingRecords});
    }
    if (!excludeNothing) {
        StreamSetBuffer * ExcludedResults = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
        kernel::Kernel * excludeK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, excludedRE, externalStreamNames);
        mGrepDriver->makeKernelCall(excludeK, {BasisBits}, {ExcludedResults});
        StreamSetBuffer * ExcludedRecords = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        mGrepDriver->makeKernelCall(matchedLinesK, {ExcludedResults, RecordBreakStream}, {ExcludedRecords});

        kernel::Kernel * invertK = mGrepDriver->addKernelInstance<kernel::InvertMatchesKernel>(idb);
        if (!matchAllLines) {
            StreamSetBuffer * nonExcluded = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
            mGrepDriver->makeKernelCall(invertK, {ExcludedRecords, RecordBreakStream}, {nonExcluded});
            StreamSetBuffer * included = MatchingRecords;
            MatchingRecords = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
            kernel::Kernel * streamsIntersectK = mGrepDriver->addKernelInstance<kernel::StreamsIntersect>(idb, 1, 2);
            mGrepDriver->makeKernelCall(streamsIntersectK, {included, nonExcluded}, {MatchingRecords});
        }
        else {
            MatchingRecords = mGrepDriver->addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize);
            mGrepDriver->makeKernelCall(invertK, {ExcludedRecords, RecordBreakStream}, {MatchingRecords});
        }
    }
    kernel::Kernel * scanMatchK = mGrepDriver->addKernelInstance<kernel::ScanMatchKernel>(idb);
    scanMatchK->setInitialArguments({ConstantInt::get(idb->getIntAddrTy(), reinterpret_cast<intptr_t>(accum))});
    mGrepDriver->makeKernelCall(scanMatchK, {MatchingRecords, RecordBreakStream, ByteStream}, {});
    mGrepDriver->LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    mGrepDriver->LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);
    mGrepDriver->generatePipelineIR();
    mGrepDriver->deallocateBuffers();
    idb->CreateRetVoid();
    mGrepDriver->finalizeObject();
}

void InternalSearchEngine::doGrep(const char * search_buffer, size_t bufferLength) {
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());
    f(search_buffer, bufferLength);
    codegen::SegmentPipelineParallel = mSaveSegmentPipelineParallel;
}

GrepEngine::~GrepEngine() { }

InternalSearchEngine::~InternalSearchEngine() { }

}
