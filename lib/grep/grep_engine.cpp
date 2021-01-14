/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep/grep_engine.h>

#include <atomic>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sched.h>
#include <boost/filesystem.hpp>
#include <toolchain/toolchain.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <llvm/Support/Debug.h>
#include <llvm/Support/Casting.h>
#include <grep/regex_passes.h>
#include <grep/resolve_properties.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/core/idisa_target.h>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/io/source_kernel.h>
#include <kernel/core/callback.h>
#include <kernel/unicode/charclasses.h>
#include <kernel/unicode/UCD_property_kernel.h>
#include <kernel/unicode/boundary_kernels.h>
#include <kernel/util/linebreak_kernel.h>
#include <kernel/streamutils/streams_merge.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/streamutils/stream_shift.h>
#include <kernel/streamutils/string_insert.h>
#include <kernel/scan/scanmatchgen.h>
#include <kernel/streamutils/until_n.h>
#include <kernel/streamutils/sentinel.h>
#include <kernel/streamutils/run_index.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/streamutils/pdep_kernel.h>
#include <kernel/io/stdout_kernel.h>
#include <pablo/pablo_kernel.h>
#include <re/adt/adt.h>
#include <re/adt/re_utility.h>
#include <re/adt/printer_re.h>
#include <re/alphabet/alphabet.h>
#include <re/cc/cc_kernel.h>
#include <re/cc/multiplex_CCs.h>
#include <re/transforms/exclude_CC.h>
#include <re/transforms/to_utf8.h>
#include <re/analysis/re_analysis.h>
#include <re/analysis/re_name_gather.h>
#include <re/analysis/collect_ccs.h>
#include <re/transforms/replaceCC.h>
#include <re/transforms/re_multiplex.h>
#include <re/unicode/casing.h>
#include <re/unicode/boundaries.h>
#include <re/unicode/re_name_resolve.h>
#include <sys/stat.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <grep/grep_toolchain.h>
#include <toolchain/toolchain.h>
#include <kernel/util/debug_display.h>
#include <util/aligned_allocator.h>

using namespace llvm;
using namespace cc;
using namespace kernel;

namespace grep {

const auto ENCODING_BITS = 8;

void GrepCallBackObject::handle_signal(unsigned s) {
    if (static_cast<GrepSignal>(s) == GrepSignal::BinaryFile) {
        mBinaryFile = true;
    } else {
        llvm::report_fatal_error("Unknown GrepSignal");
    }
}

extern "C" void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end) {
    assert ("passed a null accumulator" && accum_addr);
    reinterpret_cast<MatchAccumulator *>(accum_addr)->accumulate_match(lineNum, line_start, line_end);
}

extern "C" void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end) {
    assert ("passed a null accumulator" && accum_addr);
    reinterpret_cast<MatchAccumulator *>(accum_addr)->finalize_match(buffer_end);
}

extern "C" unsigned get_file_count_wrapper(intptr_t accum_addr) {
    assert ("passed a null accumulator" && accum_addr);
    return reinterpret_cast<MatchAccumulator *>(accum_addr)->getFileCount();
}

extern "C" size_t get_file_start_pos_wrapper(intptr_t accum_addr, unsigned fileNo) {
    assert ("passed a null accumulator" && accum_addr);
    return reinterpret_cast<MatchAccumulator *>(accum_addr)->getFileStartPos(fileNo);
}

extern "C" void set_batch_line_number_wrapper(intptr_t accum_addr, unsigned fileNo, size_t batchLine) {
    assert ("passed a null accumulator" && accum_addr);
    reinterpret_cast<MatchAccumulator *>(accum_addr)->setBatchLineNumber(fileNo, batchLine);
}

// Grep Engine construction and initialization.

GrepEngine::GrepEngine(BaseDriver &driver) :
    mSuppressFileMessages(false),
    mBinaryFilesMode(argv::Text),
    mPreferMMap(true),
    mColoring(false),
    mShowFileNames(false),
    mStdinLabel("(stdin)"),
    mShowLineNumbers(false),
    mBeforeContext(0),
    mAfterContext(0),
    mInitialTab(false),
    mCaseInsensitive(false),
    mInvertMatches(false),
    mMaxCount(0),
    mGrepStdIn(false),
    mNullMode(NullCharMode::Data),
    mGrepDriver(driver),
    mMainMethod(nullptr),
    mNextFileToGrep(0),
    mNextFileToPrint(0),
    grepMatchFound(false),
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mExternalComponents(static_cast<Component>(0)),
    mInternalComponents(static_cast<Component>(0)),
    mLineBreakStream(nullptr),
    mU8index(nullptr),
    mGCB_stream(nullptr),
    mWordBoundary_stream(nullptr),
    mUTF8_Transformer(re::NameTransformationMode::None),
    mEngineThread(pthread_self()) {}

GrepEngine::~GrepEngine() { }

QuietModeEngine::QuietModeEngine(BaseDriver &driver) : GrepEngine(driver) {
    mEngineKind = EngineKind::QuietMode;
    mMaxCount = 1;
}

MatchOnlyEngine::MatchOnlyEngine(BaseDriver & driver, bool showFilesWithMatch, bool useNullSeparators) :
    GrepEngine(driver), mRequiredCount(showFilesWithMatch) {
    mEngineKind = EngineKind::MatchOnly;
    mFileSuffix = useNullSeparators ? std::string("\0", 1) : "\n";
    mMaxCount = 1;
    mShowFileNames = true;
}

CountOnlyEngine::CountOnlyEngine(BaseDriver &driver) : GrepEngine(driver) {
    mEngineKind = EngineKind::CountOnly;
    mFileSuffix = ":";
}

EmitMatchesEngine::EmitMatchesEngine(BaseDriver &driver)
: GrepEngine(driver) {
    mEngineKind = EngineKind::EmitMatches;
    mFileSuffix = mInitialTab ? "\t:" : ":";
}

bool GrepEngine::hasComponent(Component compon_set, Component c) {
    return (static_cast<component_t>(compon_set) & static_cast<component_t>(c)) != 0;
}

void GrepEngine::GrepEngine::setComponent(Component & compon_set, Component c) {
    compon_set = static_cast<Component>(static_cast<component_t>(compon_set) | static_cast<component_t>(c));
}

void GrepEngine::setRecordBreak(GrepRecordBreakKind b) {
    mGrepRecordBreak = b;
}

namespace fs = boost::filesystem;

std::vector<std::vector<std::string>> formFileGroups(std::vector<fs::path> paths) {
    const unsigned maxFilesPerGroup = 32;
    const uintmax_t FileBatchThreshold = 4 * codegen::SegmentSize;
    std::vector<std::vector<std::string>> groups;
    // The total size of files in the current group, or 0 if the
    // the next file should start a new group.
    uintmax_t groupTotalSize = 0;
    for (auto p : paths) {
        boost::system::error_code errc;
        auto s = fs::file_size(p, errc);
        if ((s > 0) && (s < FileBatchThreshold)) {
            if (groupTotalSize == 0) {
                groups.push_back({p.string()});
                groupTotalSize = s;
            } else {
                groups.back().push_back(p.string());
                groupTotalSize += s;
                if ((groupTotalSize > FileBatchThreshold) || (groups.back().size() == maxFilesPerGroup)) {
                    // Signal to start a new group
                    groupTotalSize = 0;
                }
            }
        } else {
            // For large files, or in the case of non-regular file or other error,
            // the path is saved in its own group.
            groups.push_back({p.string()});
            // This group is done, signal to start a new group.
            groupTotalSize = 0;
        }
    }
    return groups;
}

bool GrepEngine::haveFileBatch() {
    for (auto & b : mFileGroups) {
        if (b.size() > 1) return true;
    }
    return false;
}

void GrepEngine::initFileResult(const std::vector<boost::filesystem::path> & paths) {
    const unsigned n = paths.size();
    mResultStrs.resize(n);
    mFileStatus.resize(n, FileStatus::Pending);
    mInputPaths = paths;
    mFileGroups = formFileGroups(paths);
    const unsigned numOfThreads = std::min(static_cast<unsigned>(codegen::TaskThreads),
                                           std::max(static_cast<unsigned>(mFileGroups.size()), 1u));
    codegen::setTaskThreads(numOfThreads);
}

//
// Moving matches to EOL.   Mathches need to be aligned at EOL if for
// scanning or counting processes (with a max count != 1).   If the REs
// are not all anchored, then we need to move the matches to EOL.
bool GrepEngine::matchesToEOLrequired () {
    // Moving matches is required for UnicodeLines mode, because matches
    // may be on the CR of a CRLF.
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) return true;
    // If all REs are anchored to EOL already, then we can avoid moving them.
    bool allAnchored = true;
    for (unsigned i = 0; i < mREs.size(); ++i) {
        if (!hasEndAnchor(mREs[i])) allAnchored = false;
    }
    if (allAnchored) return false;
    //
    // Not all REs are anchored.   We can avoid moving matches, if we are
    // in MatchOnly mode (or CountOnly with MaxCount = 1) and no invert match inversion.
    return (mEngineKind == EngineKind::EmitMatches) || (mMaxCount != 1) || mInvertMatches;
}

void GrepEngine::initREs(std::vector<re::RE *> & REs) {
    if (mEngineKind != EngineKind::EmitMatches) mColoring = false;
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        mBreakCC = re::makeCC(re::makeCC(0x0A, 0x0D), re::makeCC(re::makeCC(0x85), re::makeCC(0x2028, 0x2029)));
        for (unsigned i = 0; i < REs.size(); ++i) {
            if (hasEndAnchor(REs[i])) {
                UnicodeIndexing = true;
                break;
            }
        }
    } else if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        mBreakCC = re::makeCC(0, &cc::Unicode);  // Null
    } else {
        mBreakCC = re::makeCC(0x0A, &cc::Unicode); // LF
    }
    re::RE * anchorRE = mBreakCC;
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        re::Name * anchorName = re::makeName("UTF8_LB", re::Name::Type::Unicode);
        anchorName->setDefinition(re::makeUnicodeBreak());
        anchorRE = anchorName;
        setComponent(mExternalComponents, Component::UTF8index);
        mExternalNames.insert(anchorName);
    }

    mREs = REs;
    for (unsigned i = 0; i < mREs.size(); ++i) {
        mREs[i] = resolveModesAndExternalSymbols(mREs[i], mCaseInsensitive);
        mREs[i] = re::exclude_CC(mREs[i], mBreakCC);
        mREs[i] = resolveAnchors(mREs[i], anchorRE);
        mREs[i] = regular_expression_passes(mREs[i]);
    }
    for (unsigned i = 0; i < mREs.size(); ++i) {
        if (hasGraphemeClusterBoundary(mREs[i])) {
            UnicodeIndexing = true;
            setComponent(mExternalComponents, Component::GraphemeClusterBoundary);
            break;
        }
    }
    for (unsigned i = 0; i < mREs.size(); ++i) {
        if (hasWordBoundary(mREs[i])) {
            UnicodeIndexing = true;
            setComponent(mExternalComponents, Component::WordBoundary);
            break;
        }
    }
    for (unsigned i = 0; i < mREs.size(); ++i) {
        if (!validateFixedUTF8(mREs[i])) {
            setComponent(mExternalComponents, Component::UTF8index);
            if (mColoring) {
                UnicodeIndexing = true;
            }
            break;
        }
    }
    if (UnicodeIndexing) {
        setComponent(mExternalComponents, Component::S2P);
        setComponent(mExternalComponents, Component::UTF8index);
    }
    if ((mEngineKind == EngineKind::EmitMatches) && mColoring && !mInvertMatches) {
        setComponent(mExternalComponents, Component::MatchStarts);
    }
    if (matchesToEOLrequired()) {
        // Move matches to EOL.   This may be achieved internally by modifying
        // the regular expression or externally.   The internal approach is more
        // generally more efficient, but cannot be used if colorization is needed
        // or in UnicodeLines mode.
        if ((mGrepRecordBreak == GrepRecordBreakKind::Unicode) || (mEngineKind == EngineKind::EmitMatches) || mInvertMatches || UnicodeIndexing) {
            setComponent(mExternalComponents, Component::MoveMatchesToEOL);
        } else {
            setComponent(mInternalComponents, Component::MoveMatchesToEOL);
        }
    }
    if (hasComponent(mInternalComponents, Component::MoveMatchesToEOL)) {
        re::RE * notBreak = re::makeDiff(re::makeByte(0x00, 0xFF), toUTF8(mBreakCC));
        for (unsigned i = 0; i < mREs.size(); ++i) {
            if (!hasEndAnchor(mREs[i])) {
                mREs[i] = re::makeSeq({mREs[i], re::makeRep(notBreak, 0, re::Rep::UNBOUNDED_REP), makeNegativeLookAheadAssertion(notBreak)});
            }
        }
    }
    for (unsigned i = 0; i < mREs.size(); ++i) {
        re::gatherNames(mREs[i], mExternalNames);
    }

    // For simple regular expressions with a small number of characters, we
    // can bypass transposition and use the Direct CC compiler.
    mPrefixRE = nullptr;
    mSuffixRE = nullptr;
    if ((mREs.size() == 1) && (mGrepRecordBreak != GrepRecordBreakKind::Unicode) &&
        mExternalNames.empty() && !UnicodeIndexing) {
        if (byteTestsWithinLimit(mREs[0], ByteCClimit)) {
            return;  // skip transposition
        } else if (hasTriCCwithinLimit(mREs[0], ByteCClimit, mPrefixRE, mSuffixRE)) {
            return;  // skip transposition and set mPrefixRE, mSuffixRE
        } else {
            setComponent(mExternalComponents, Component::S2P);
        }
    } else {
        setComponent(mExternalComponents, Component::S2P);
    }
    if (!mExternalNames.empty()) {
        setComponent(mExternalComponents, Component::UTF8index);
    }
}

StreamSet * GrepEngine::getBasis(const std::unique_ptr<ProgramBuilder> & P, StreamSet * ByteStream) {
    if (hasComponent(mExternalComponents, Component::S2P)) {
        StreamSet * BasisBits = P->CreateStreamSet(ENCODING_BITS, 1);
        if (PabloTransposition) {
            P->CreateKernelCall<S2P_PabloKernel>(ByteStream, BasisBits);
        } else {
            P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
        }
        return BasisBits;
    }
    else return ByteStream;
}

void GrepEngine::grepPrologue(const std::unique_ptr<ProgramBuilder> & P, StreamSet * SourceStream) {

    mLineBreakStream = nullptr;
    mU8index = nullptr;
    mGCB_stream = nullptr;
    mWordBoundary_stream = nullptr;
    mPropertyStreamMap.clear();

    Scalar * const callbackObject = P->getInputScalar("callbackObject");
    if (mBinaryFilesMode == argv::Text) {
        mNullMode = NullCharMode::Data;
    } else if (mBinaryFilesMode == argv::WithoutMatch) {
        mNullMode = NullCharMode::Abort;
    } else {
        mNullMode = NullCharMode::Break;
    }
    mLineBreakStream = P->CreateStreamSet(1, 1);
    mU8index = P->CreateStreamSet(1, 1);
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        UnicodeLinesLogic(P, SourceStream, mLineBreakStream, mU8index, UnterminatedLineAtEOF::Add1, mNullMode, callbackObject);
    }
    else {
        if (hasComponent(mExternalComponents, Component::UTF8index)) {
            P->CreateKernelCall<UTF8_index>(SourceStream, mU8index);
        }
        if (mGrepRecordBreak == GrepRecordBreakKind::LF) {
            Kernel * k = P->CreateKernelCall<UnixLinesKernelBuilder>(SourceStream, mLineBreakStream, UnterminatedLineAtEOF::Add1, mNullMode, callbackObject);
            if (mNullMode == NullCharMode::Abort) {
                k->link("signal_dispatcher", kernel::signal_dispatcher);
            }
        } else { // if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
            P->CreateKernelCall<NullDelimiterKernel>(SourceStream, mLineBreakStream, UnterminatedLineAtEOF::Add1);
        }
    }
}

void GrepEngine::prepareExternalStreams(const std::unique_ptr<ProgramBuilder> & P, StreamSet * SourceStream) {
    if (hasComponent(mExternalComponents, Component::GraphemeClusterBoundary)) {
        mGCB_stream = P->CreateStreamSet(1, 1);
        GraphemeClusterLogic(P, &mUTF8_Transformer, SourceStream, mU8index, mGCB_stream);
    }
    if (hasComponent(mExternalComponents, Component::WordBoundary)) {
        mWordBoundary_stream = P->CreateStreamSet(1, 1);
        WordBoundaryLogic(P, &mUTF8_Transformer, SourceStream, mU8index, mWordBoundary_stream);
    }
    if (PropertyKernels) {
        for (auto e : mExternalNames) {
            if (isa<re::CC>(e->getDefinition())) {
                auto name = e->getFullName();
                StreamSet * property = P->CreateStreamSet(1, 1);
                //errs() << "preparing external: " << name << "\n";
                mPropertyStreamMap.emplace(name, property);
                P->CreateKernelCall<UnicodePropertyKernelBuilder>(e, SourceStream, property);
            }
        }
    }
}


void GrepEngine::addExternalStreams(const std::unique_ptr<ProgramBuilder> & P, std::unique_ptr<GrepKernelOptions> & options, re::RE * regexp, StreamSet * indexMask) {
    std::set<re::Name *> externals;
    re::gatherNames(regexp, externals);
    for (const auto & e : externals) {
        auto name = e->getFullName();
        const auto f = mPropertyStreamMap.find(name);
        if (f != mPropertyStreamMap.end()) {
            //errs() << "adding external: " << name << "\n";
            if (indexMask == nullptr) {
                options->addExternal(name, f->second);
            } else {
                StreamSet * iExternal_stream = P->CreateStreamSet(1, 1);
                FilterByMask(P, indexMask, f->second, iExternal_stream);
                options->addExternal(name, iExternal_stream);
            }
        }
    }
    if (hasComponent(mExternalComponents, Component::GraphemeClusterBoundary)) {
        assert (mGCB_stream);
        if (indexMask == nullptr) {
            options->addExternal("\\b{g}", mGCB_stream);
            //P->CreateKernelCall<DebugDisplayKernel>("\\b{g}[U8indexed]", mGCB_stream);
        } else {
            StreamSet * iGCB_stream = P->CreateStreamSet(1, 1);
            FilterByMask(P, indexMask, mGCB_stream, iGCB_stream);
            options->addExternal("\\b{g}", iGCB_stream, 1);
            //P->CreateKernelCall<DebugDisplayKernel>("\\b{g}", iGCB_stream);
        }
    }
    if (hasComponent(mExternalComponents, Component::WordBoundary)) {
        assert (mWordBoundary_stream);
        if (indexMask == nullptr) {
            options->addExternal("\\b", mWordBoundary_stream);
            //P->CreateKernelCall<DebugDisplayKernel>("\\b[U8indexed]", mWordBoundary_stream);
        } else {
            StreamSet * iWordBoundary_stream = P->CreateStreamSet(1, 1);
            FilterByMask(P, indexMask, mWordBoundary_stream, iWordBoundary_stream);
            options->addExternal("\\b", iWordBoundary_stream, 1);
            //P->CreateKernelCall<DebugDisplayKernel>("\\b", iWordBoundary_stream);
        }
    }
    if (hasComponent(mExternalComponents, Component::UTF8index)) {
        if (indexMask == nullptr) {
            options->addExternal("UTF8_index", mU8index);
        }
    }
    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        if (indexMask == nullptr) {
            options->addExternal("UTF8_LB", mLineBreakStream);
        } else {
            StreamSet * iU8_LB = P->CreateStreamSet(1, 1);
            FilterByMask(P, indexMask, mLineBreakStream, iU8_LB);
            options->addExternal("UTF8_LB", iU8_LB, 1);
        }
    }
}

void GrepEngine::UnicodeIndexedGrep(const std::unique_ptr<ProgramBuilder> & P, re::RE * re, StreamSet * Source, StreamSet * Results) {
    std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>(&cc::Unicode);
    auto lengths = getLengthRange(re, &cc::Unicode);
    const auto UnicodeSets = re::collectCCs(re, cc::Unicode, mExternalNames);
    if (UnicodeSets.empty()) {
        // All inputs will be externals.   Do not register a source stream.
        options->setRE(re);
    } else {
        auto mpx = std::make_shared<MultiplexedAlphabet>("mpx", UnicodeSets);
        re = transformCCs(mpx, re, mExternalNames);
        options->setRE(re);
        auto mpx_basis = mpx->getMultiplexedCCs();
        StreamSet * const u8CharClasses = P->CreateStreamSet(mpx_basis.size());
        StreamSet * const CharClasses = P->CreateStreamSet(mpx_basis.size());
        P->CreateKernelCall<CharClassesKernel>(std::move(mpx_basis), Source, u8CharClasses);
        FilterByMask(P, mU8index, u8CharClasses, CharClasses);
        options->setSource(CharClasses);
        options->addAlphabet(mpx, CharClasses);
    }
    StreamSet * const MatchResults = P->CreateStreamSet(1, 1);
    options->setResults(MatchResults);
    addExternalStreams(P, options, re, mU8index);
    P->CreateKernelCall<ICGrepKernel>(std::move(options));
    StreamSet * u8index1 = P->CreateStreamSet(1, 1);
    P->CreateKernelCall<AddSentinel>(mU8index, u8index1);
    if (hasComponent(mExternalComponents, Component::MatchStarts)) {
        StreamSet * u8initial = P->CreateStreamSet(1, 1);
        P->CreateKernelCall<LineStartsKernel>(mU8index, u8initial);
        StreamSet * MatchPairs = P->CreateStreamSet(2, 1);
        P->CreateKernelCall<FixedMatchPairsKernel>(lengths.first, MatchResults, MatchPairs);
        SpreadByMask(P, u8initial, MatchPairs, Results);
    } else {
        SpreadByMask(P, u8index1, MatchResults, Results);
    }
}

void GrepEngine::U8indexedGrep(const std::unique_ptr<ProgramBuilder> & P, re::RE * re, StreamSet * Source, StreamSet * Results) {
    std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>(&cc::UTF8);
    auto lengths = getLengthRange(re, &cc::UTF8);
    options->setSource(Source);
    StreamSet * MatchResults = nullptr;
    if (hasComponent(mExternalComponents, Component::MatchStarts)) {
        MatchResults = P->CreateStreamSet(1, 1);
        options->setResults(MatchResults);
    } else {
        options->setResults(Results);
    }
    if (hasComponent(mExternalComponents, Component::UTF8index)) {
        options->setIndexingTransformer(&mUTF8_Transformer, mU8index);
        if (mSuffixRE != nullptr) {
            options->setPrefixRE(mPrefixRE);
            options->setRE(mSuffixRE);
        } else {
            options->setRE(re);
        }
    } else {
        if (mSuffixRE != nullptr) {
            options->setPrefixRE(toUTF8(mPrefixRE));
            options->setRE(toUTF8(mSuffixRE));
        } else {
            options->setRE(toUTF8(re));
        }
    }
    addExternalStreams(P, options, re);
    P->CreateKernelCall<ICGrepKernel>(std::move(options));
    if (hasComponent(mExternalComponents, Component::MatchStarts)) {
        P->CreateKernelCall<FixedMatchPairsKernel>(lengths.first, MatchResults, Results);
    }
}

StreamSet * GrepEngine::grepPipeline(const std::unique_ptr<ProgramBuilder> & P, StreamSet * InputStream) {
    StreamSet * SourceStream = getBasis(P, InputStream);

    grepPrologue(P, SourceStream);

    prepareExternalStreams(P, SourceStream);

    const auto numOfREs = mREs.size();
    std::vector<StreamSet *> MatchResultsBufs(numOfREs);

    for(unsigned i = 0; i < numOfREs; ++i) {
        StreamSet * const MatchResults = P->CreateStreamSet(1, 1);
        MatchResultsBufs[i] = MatchResults;
        if (UnicodeIndexing) {
            UnicodeIndexedGrep(P, mREs[i], SourceStream, MatchResults);
        } else {
            U8indexedGrep(P, mREs[i], SourceStream, MatchResults);
        }
    }

    StreamSet * Matches = MatchResultsBufs[0];
    if (MatchResultsBufs.size() > 1) {
        StreamSet * const MergedMatches = P->CreateStreamSet();
        P->CreateKernelCall<StreamsMerge>(MatchResultsBufs, MergedMatches);
        Matches = MergedMatches;
    }
    if (hasComponent(mExternalComponents, Component::MoveMatchesToEOL)) {
        StreamSet * const MovedMatches = P->CreateStreamSet();
        P->CreateKernelCall<MatchedLinesKernel>(Matches, mLineBreakStream, MovedMatches);
        Matches = MovedMatches;
    }
    if (mInvertMatches) {
        StreamSet * const InvertedMatches = P->CreateStreamSet();
        P->CreateKernelCall<InvertMatchesKernel>(Matches, mLineBreakStream, InvertedMatches);
        Matches = InvertedMatches;
    }
    if (mMaxCount > 0) {
        StreamSet * const TruncatedMatches = P->CreateStreamSet();
        Scalar * const maxCount = P->getInputScalar("maxCount");
        P->CreateKernelCall<UntilNkernel>(maxCount, Matches, TruncatedMatches);
        Matches = TruncatedMatches;
    }
    return Matches;
}



// The QuietMode, MatchOnly and CountOnly engines share a common code generation main function,
// which returns a count of the matches found (possibly subject to a MaxCount).
//

void GrepEngine::grepCodeGen() {
    auto & idb = mGrepDriver.getBuilder();

    auto P = mGrepDriver.makePipeline(
                // inputs
                {Binding{idb->getSizeTy(), "useMMap"},
                Binding{idb->getInt32Ty(), "fileDescriptor"},
                Binding{idb->getIntAddrTy(), "callbackObject"},
                Binding{idb->getSizeTy(), "maxCount"}}
                ,// output
                {Binding{idb->getInt64Ty(), "countResult"}});

    Scalar * const useMMap = P->getInputScalar("useMMap");
    Scalar * const fileDescriptor = P->getInputScalar("fileDescriptor");

    StreamSet * const ByteStream = P->CreateStreamSet(1, ENCODING_BITS);
    P->CreateKernelCall<FDSourceKernel>(useMMap, fileDescriptor, ByteStream);
    StreamSet * const Matches = grepPipeline(P, ByteStream);
    P->CreateKernelCall<PopcountKernel>(Matches, P->getOutputScalar("countResult"));

    mMainMethod = P->compile();
}

//
//  Default Report Match:  lines are emitted with whatever line terminators are found in the
//  input.  However, if the final line is not terminated, a new line is appended.
//
const size_t batch_alignment = 64;

void EmitMatch::setFileLabel(std::string fileLabel) {
    if (mShowFileNames) {
        mLinePrefix = fileLabel + (mInitialTab ? "\t:" : ":");
    } else mLinePrefix = "";
}

void EmitMatch::setStringStream(std::ostringstream * s) {
    mResultStr = s;
}

unsigned EmitMatch::getFileCount() {
    mCurrentFile = 0;
    if (mFileNames.size() == 0) return 1;
    return mFileNames.size();
}

size_t EmitMatch::getFileStartPos(unsigned fileNo) {
    if (mFileStartPositions.size() == 0) return 0;
    assert(fileNo < mFileStartPositions.size());
    //llvm::errs() << "getFileStartPos(" << fileNo << ") = ";
    //llvm::errs().write_hex(mFileStartPositions[fileNo]);
    //llvm::errs() << "  file = " << mFileNames[fileNo] << "\n";
    return mFileStartPositions[fileNo];
}

void EmitMatch::setBatchLineNumber(unsigned fileNo, size_t batchLine) {
    //llvm::errs() << "setBatchLineNumber(" << fileNo << ", " << batchLine << ")  file = " << mFileNames[fileNo] << "\n";
    mFileStartLineNumbers[fileNo+1] = batchLine;
    if (!mTerminated) *mResultStr << "\n";
    mTerminated = true;
}

void EmitMatch::accumulate_match (const size_t lineNum, char * line_start, char * line_end) {
    //llvm::errs() << "lineNum = " << lineNum << "\n";
    while ((mCurrentFile + 1 < mFileStartPositions.size()) && (mFileStartLineNumbers[mCurrentFile + 1] <= lineNum)) {
        mCurrentFile++;
        //llvm::errs() << "mCurrentFile = " << mCurrentFile << "\n";
        setFileLabel(mFileNames[mCurrentFile]);
    }
    size_t relLineNum = mCurrentFile > 0 ? lineNum - mFileStartLineNumbers[mCurrentFile] : lineNum;
    if (mContextGroups && (lineNum > mLineNum + 1) && (relLineNum > 0)) {
        *mResultStr << "--\n";
    }
    *mResultStr << mLinePrefix;
    if (mShowLineNumbers) {
        // Internally line numbers are counted from 0.  For display, adjust
        // the line number so that lines are numbered from 1.
        if (mInitialTab) {
            *mResultStr << relLineNum+1 << "\t:";
        }
        else {
            *mResultStr << relLineNum+1 << ":";
        }
    }

    const auto bytes = line_end - line_start + 1;
    mResultStr->write(line_start, bytes);
    mLineCount++;
    mLineNum = lineNum;
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
    if (!mTerminated) *mResultStr << "\n";
}

void EmitMatchesEngine::grepPipeline(const std::unique_ptr<ProgramBuilder> & E, StreamSet * ByteStream, bool BatchMode) {
    StreamSet * SourceStream = getBasis(E, ByteStream);

    grepPrologue(E, SourceStream);

    prepareExternalStreams(E, SourceStream);

    const auto numOfREs = mREs.size();
    std::vector<StreamSet *> MatchResultsBufs(numOfREs);

    unsigned matchResultStreamCount = (mColoring && !mInvertMatches) ? 2 : 1;
    for(unsigned i = 0; i < numOfREs; ++i) {
        StreamSet * const MatchResults = E->CreateStreamSet(matchResultStreamCount, 1);
        MatchResultsBufs[i] = MatchResults;
        if (UnicodeIndexing) {
            UnicodeIndexedGrep(E, mREs[i], SourceStream, MatchResults);
        } else {
            U8indexedGrep(E, mREs[i], SourceStream, MatchResults);
        }
    }
    StreamSet * Matches = MatchResultsBufs[0];
    if (MatchResultsBufs.size() > 1) {
        StreamSet * const MergedMatches = E->CreateStreamSet(matchResultStreamCount);
        E->CreateKernelCall<StreamsMerge>(MatchResultsBufs, MergedMatches);
        Matches = MergedMatches;
    }
    StreamSet * MatchedLineEnds;
    if (hasComponent(mExternalComponents, Component::MatchStarts)) {
        StreamSet * Selected = E->CreateStreamSet(1, 1);
        E->CreateKernelCall<StreamSelect>(Selected, Select(Matches, {1}));
        MatchedLineEnds = Selected;
    } else {
        MatchedLineEnds = Matches;
    }

    if (hasComponent(mExternalComponents, Component::MoveMatchesToEOL)) {
        StreamSet * const MovedMatches = E->CreateStreamSet();
        E->CreateKernelCall<MatchedLinesKernel>(MatchedLineEnds, mLineBreakStream, MovedMatches);
        MatchedLineEnds = MovedMatches;
    }
    if (mInvertMatches) {
        StreamSet * const InvertedMatches = E->CreateStreamSet();
        E->CreateKernelCall<InvertMatchesKernel>(MatchedLineEnds, mLineBreakStream, InvertedMatches);
        MatchedLineEnds = InvertedMatches;
    }
    if (mMaxCount > 0) {
        StreamSet * const TruncatedMatches = E->CreateStreamSet();
        Scalar * const maxCount = E->getInputScalar("maxCount");
        E->CreateKernelCall<UntilNkernel>(maxCount, MatchedLineEnds, TruncatedMatches);
        MatchedLineEnds = TruncatedMatches;
    }

    if (mColoring && !mInvertMatches) {

        StreamSet * MatchesByLine = E->CreateStreamSet(1, 1);
        FilterByMask(E, mLineBreakStream, MatchedLineEnds, MatchesByLine);

        if ((mAfterContext != 0) || (mBeforeContext != 0)) {
            StreamSet * ContextByLine = E->CreateStreamSet(1, 1);
            E->CreateKernelCall<ContextSpan>(MatchesByLine, ContextByLine, mBeforeContext, mAfterContext);
            StreamSet * SelectedLines = E->CreateStreamSet(1, 1);
            SpreadByMask(E, mLineBreakStream, ContextByLine, SelectedLines);
            MatchedLineEnds = SelectedLines;
            MatchesByLine = ContextByLine;
        }

        StreamSet * SourceCoords = nullptr;
        if (BatchMode) {
            //llvm::errs() << "Batch mode calling BatchCoordinatesKernel\n";
            SourceCoords = E->CreateStreamSet(1, sizeof(size_t) * 8);
            Scalar * const callbackObject = E->getInputScalar("callbackObject");
            Kernel * const batchK = E->CreateKernelCall<BatchCoordinatesKernel>(MatchedLineEnds, mLineBreakStream, SourceCoords, callbackObject);
            batchK->link("get_file_count_wrapper", get_file_count_wrapper);
            batchK->link("get_file_start_pos_wrapper", get_file_start_pos_wrapper);
            batchK->link("set_batch_line_number_wrapper", set_batch_line_number_wrapper);
            //E->CreateKernelCall<DebugDisplayKernel>("SourceCoords", SourceCoords);
        } else {
            SourceCoords = E->CreateStreamSet(3, sizeof(size_t) * 8);
            E->CreateKernelCall<MatchCoordinatesKernel>(MatchedLineEnds, mLineBreakStream, SourceCoords, 1);
        }

        StreamSet * LineStarts = E->CreateStreamSet(1, 1);
        E->CreateKernelCall<LineStartsKernel>(mLineBreakStream, LineStarts);
        StreamSet * MatchedLineStarts = E->CreateStreamSet(1, 1);
        SpreadByMask(E, LineStarts, MatchesByLine, MatchedLineStarts);

        StreamSet * Filtered = E->CreateStreamSet(1, 8);
        E->CreateKernelCall<MatchFilterKernel>(MatchedLineStarts, mLineBreakStream, ByteStream, Filtered);

        StreamSet * MatchedLineSpans = E->CreateStreamSet(1, 1);
        E->CreateKernelCall<LineSpansKernel>(MatchedLineStarts, MatchedLineEnds, MatchedLineSpans);
        //E->CreateKernelCall<DebugDisplayKernel>("MatchedLineSpans", MatchedLineSpans);

        StreamSet * InsertMarks = E->CreateStreamSet(2, 1);
        FilterByMask(E, MatchedLineSpans, Matches, InsertMarks);
        //E->CreateKernelCall<DebugDisplayKernel>("Matches", Matches);

        std::string ESC = "\x1B";
        std::vector<std::string> colorEscapes = {ESC + "[01;31m" + ESC + "[K", ESC + "[m"};
        unsigned insertLengthBits = 4;

        StreamSet * const InsertBixNum = E->CreateStreamSet(insertLengthBits, 1);
        E->CreateKernelCall<StringInsertBixNum>(colorEscapes, InsertMarks, InsertBixNum);
        //E->CreateKernelCall<DebugDisplayKernel>("InsertBixNum", InsertBixNum);
        StreamSet * const SpreadMask = InsertionSpreadMask(E, InsertBixNum, InsertPosition::Before);
        //E->CreateKernelCall<DebugDisplayKernel>("SpreadMask", SpreadMask);

        // For each run of 0s marking insert positions, create a parallel
        // bixnum sequentially numbering the string insert positions.
        StreamSet * const InsertIndex = E->CreateStreamSet(insertLengthBits);
        E->CreateKernelCall<RunIndex>(SpreadMask, InsertIndex, nullptr, /*invert = */ true);
        //E->CreateKernelCall<DebugDisplayKernel>("InsertIndex", InsertIndex);

        StreamSet * FilteredBasis = E->CreateStreamSet(8, 1);
        E->CreateKernelCall<S2PKernel>(Filtered, FilteredBasis);

        // Baais bit streams expanded with 0 bits for each string to be inserted.
        StreamSet * ExpandedBasis = E->CreateStreamSet(8);
        SpreadByMask(E, SpreadMask, FilteredBasis, ExpandedBasis);
        //E->CreateKernelCall<DebugDisplayKernel>("ExpandedBasis", ExpandedBasis);

        // Map the match start/end marks to their positions in the expanded basis.
        StreamSet * ExpandedMarks = E->CreateStreamSet(2);
        SpreadByMask(E, SpreadMask, InsertMarks, ExpandedMarks);

        StreamSet * ColorizedBasis = E->CreateStreamSet(8);
        E->CreateKernelCall<StringReplaceKernel>(colorEscapes, ExpandedBasis, SpreadMask, ExpandedMarks, InsertIndex, ColorizedBasis);

        StreamSet * ColorizedBytes  = E->CreateStreamSet(1, 8);
        E->CreateKernelCall<P2SKernel>(ColorizedBasis, ColorizedBytes);

        StreamSet * ColorizedBreaks = E->CreateStreamSet(1);
        E->CreateKernelCall<UnixLinesKernelBuilder>(ColorizedBasis, ColorizedBreaks);

        StreamSet * ColorizedCoords = E->CreateStreamSet(3, sizeof(size_t) * 8);
        E->CreateKernelCall<MatchCoordinatesKernel>(ColorizedBreaks, ColorizedBreaks, ColorizedCoords, 1);

        Scalar * const callbackObject = E->getInputScalar("callbackObject");
        Kernel * const matchK = E->CreateKernelCall<ColorizedReporter>(ColorizedBytes, SourceCoords, ColorizedCoords, callbackObject);
        matchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
        matchK->link("finalize_match_wrapper", finalize_match_wrapper);
    } else { // Non colorized output
        if ((mAfterContext != 0) || (mBeforeContext != 0)) {
            StreamSet * MatchesByLine = E->CreateStreamSet(1, 1);
            FilterByMask(E, mLineBreakStream, MatchedLineEnds, MatchesByLine);
            StreamSet * ContextByLine = E->CreateStreamSet(1, 1);
            E->CreateKernelCall<ContextSpan>(MatchesByLine, ContextByLine, mBeforeContext, mAfterContext);
            StreamSet * SelectedLines = E->CreateStreamSet(1, 1);
            SpreadByMask(E, mLineBreakStream, ContextByLine, SelectedLines);
            MatchedLineEnds = SelectedLines;
        }
        if (MatchCoordinateBlocks > 0) {
            StreamSet * MatchCoords = E->CreateStreamSet(3, sizeof(size_t) * 8);
            E->CreateKernelCall<MatchCoordinatesKernel>(MatchedLineEnds, mLineBreakStream, MatchCoords, MatchCoordinateBlocks);
            Scalar * const callbackObject = E->getInputScalar("callbackObject");
            Kernel * const matchK = E->CreateKernelCall<MatchReporter>(ByteStream, MatchCoords, callbackObject);
            matchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
            matchK->link("finalize_match_wrapper", finalize_match_wrapper);
        } else {
            if (BatchMode) {
                Scalar * const callbackObject = E->getInputScalar("callbackObject");
                Kernel * const scanBatchK = E->CreateKernelCall<ScanBatchKernel>(MatchedLineEnds, mLineBreakStream, ByteStream, callbackObject, ScanMatchBlocks);
                scanBatchK->link("get_file_count_wrapper", get_file_count_wrapper);
                scanBatchK->link("get_file_start_pos_wrapper", get_file_start_pos_wrapper);
                scanBatchK->link("set_batch_line_number_wrapper", set_batch_line_number_wrapper);
                scanBatchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
                scanBatchK->link("finalize_match_wrapper", finalize_match_wrapper);
            } else {
                Scalar * const callbackObject = E->getInputScalar("callbackObject");
                Kernel * const matchK = E->CreateKernelCall<ScanMatchKernel>(MatchedLineEnds, mLineBreakStream, ByteStream, callbackObject, ScanMatchBlocks);
                matchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
                matchK->link("finalize_match_wrapper", finalize_match_wrapper);
            }
        }
    }
    //E->CreateKernelCall<StdOutKernel>(ColorizedBytes);
}

void EmitMatchesEngine::grepCodeGen() {
    auto & idb = mGrepDriver.getBuilder();

    auto E1 = mGrepDriver.makePipeline(
                // inputs
                {Binding{idb->getSizeTy(), "useMMap"},
                Binding{idb->getInt32Ty(), "fileDescriptor"},
                Binding{idb->getIntAddrTy(), "callbackObject"},
                Binding{idb->getSizeTy(), "maxCount"}}
                ,// output
                {Binding{idb->getInt64Ty(), "countResult"}});

    Scalar * const useMMap = E1->getInputScalar("useMMap");
    Scalar * const fileDescriptor = E1->getInputScalar("fileDescriptor");
    StreamSet * const ByteStream = E1->CreateStreamSet(1, ENCODING_BITS);
    E1->CreateKernelCall<FDSourceKernel>(useMMap, fileDescriptor, ByteStream);
    grepPipeline(E1, ByteStream);
    E1->setOutputScalar("countResult", E1->CreateConstant(idb->getInt64(0)));
    mMainMethod = E1->compile();

    if (haveFileBatch()) {
        auto E2 = mGrepDriver.makePipeline(
                    // inputs
                    {Binding{idb->getInt8PtrTy(), "buffer"},
                    Binding{idb->getSizeTy(), "length"},
                    Binding{idb->getIntAddrTy(), "callbackObject"},
                    Binding{idb->getSizeTy(), "maxCount"}}
                    ,// output
                    {Binding{idb->getInt64Ty(), "countResult"}});

        Scalar * const buffer = E2->getInputScalar("buffer");
        Scalar * const length = E2->getInputScalar("length");
        StreamSet * const InternalBytes = E2->CreateStreamSet(1, 8);
        E2->CreateKernelCall<MemorySourceKernel>(buffer, length, InternalBytes);
        grepPipeline(E2, InternalBytes, /* BatchMode = */ true);
        E2->setOutputScalar("countResult", E2->CreateConstant(idb->getInt64(0)));
        mBatchMethod = E2->compile();
    }
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


uint64_t GrepEngine::doGrep(const std::vector<std::string> & fileNames, std::ostringstream & strm) {
    typedef uint64_t (*GrepFunctionType)(bool useMMap, int32_t fileDescriptor, GrepCallBackObject *, size_t maxCount);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);
    GrepCallBackObject handler;
    uint64_t resultTotal = 0;

    for (auto fileName : fileNames) {
        bool useMMap = mPreferMMap && canMMap(fileName);
        int32_t fileDescriptor = openFile(fileName, strm);
        if (fileDescriptor == -1) return 0;
        uint64_t grepResult = f(useMMap, fileDescriptor, &handler, mMaxCount);

        close(fileDescriptor);
        if (handler.binaryFileSignalled()) {
            llvm::errs() << "Binary file " << fileName << "\n";
        }
        else {
            showResult(grepResult, fileName, strm);
            resultTotal += grepResult;
        }
    }
    return resultTotal;
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

uint64_t EmitMatchesEngine::doGrep(const std::vector<std::string> & fileNames, std::ostringstream & strm) {
    if (fileNames.size() == 1) {
        typedef uint64_t (*GrepFunctionType)(bool useMMap, int32_t fileDescriptor, EmitMatch *, size_t maxCount);
        auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);
        EmitMatch accum(mShowFileNames, mShowLineNumbers, ((mBeforeContext > 0) || (mAfterContext > 0)), mInitialTab);
        accum.setStringStream(&strm);
        bool useMMap;
        int32_t fileDescriptor;
        if (fileNames[0] == "-") {
            fileDescriptor = STDIN_FILENO;
            accum.setFileLabel(mStdinLabel);
            useMMap = false;
        } else {
            fileDescriptor= openFile(fileNames[0], strm);
            if (fileDescriptor == -1) return 0;
            accum.setFileLabel(fileNames[0]);
            useMMap = mPreferMMap && canMMap(fileNames[0]);
        }
        f(useMMap, fileDescriptor, &accum, mMaxCount);
        close(fileDescriptor);
        if (accum.binaryFileSignalled()) {
            accum.mResultStr->clear();
            accum.mResultStr->str("");
        }
        if (accum.mLineCount > 0) grepMatchFound = true;
        return accum.mLineCount;
    } else {
        //llvm::errs() << "filenames.size() = " << fileNames.size() << "\n";
        //for (auto & name : fileNames) { llvm::errs() << name << "\n";}
        typedef uint64_t (*GrepBatchFunctionType)(char * buffer, size_t length, EmitMatch *, size_t maxCount);
        auto f = reinterpret_cast<GrepBatchFunctionType>(mBatchMethod);
        EmitMatch accum(mShowFileNames, mShowLineNumbers, ((mBeforeContext > 0) || (mAfterContext > 0)), mInitialTab);
        accum.setStringStream(&strm);
        std::vector<int32_t> fileDescriptor(fileNames.size());
        std::vector<size_t> fileSize(fileNames.size(), 0);
        size_t cumulativeSize = 0;
        unsigned filesOpened = 0;
        for (unsigned i = 0; i < fileNames.size(); i++) {
            fileDescriptor[i] = openFile(fileNames[i], strm);
            if (fileDescriptor[i] == -1) continue;  // File error; skip.
            struct stat st;
            if (fstat(fileDescriptor[i], &st) != 0) continue;
            fileSize[i] = st.st_size;
            cumulativeSize += st.st_size;
            filesOpened++;
        }
        cumulativeSize += filesOpened;  // Add an extra byte per file for possible '\n'.
        size_t aligned_size = (cumulativeSize + batch_alignment - 1) & -batch_alignment;

        AlignedAllocator<char, batch_alignment> alloc;
        accum.mBatchBuffer = alloc.allocate(aligned_size, 0);
        if (accum.mBatchBuffer == nullptr) {
            llvm::report_fatal_error("Unable to allocate batch buffer of size: " + std::to_string(aligned_size));
        }
        char * current_base = accum.mBatchBuffer;
        size_t current_start_position = 0;
        accum.mFileNames.reserve(filesOpened);
        accum.mFileStartPositions.reserve(filesOpened);

        for (unsigned i = 0; i < fileNames.size(); i++) {
            if (fileDescriptor[i] == -1) continue;  // Error opening file; skip.
            ssize_t bytes_read = read(fileDescriptor[i], current_base, fileSize[i]);
            close(fileDescriptor[i]);
            if (bytes_read <= 0) continue; // No data or error reading the file; skip.
            if (mBinaryFilesMode == argv::WithoutMatch) {
                auto null_byte_ptr = memchr(current_base, char (0), bytes_read);
                if (null_byte_ptr != nullptr) {
                    continue;  // Binary file; skip.
                }
            }
            accum.mFileNames.push_back(fileNames[i]);
            accum.mFileStartPositions.push_back(current_start_position);
            current_base += bytes_read;
            current_start_position += bytes_read;
            if (*(current_base - 1) != '\n') {
                *current_base = '\n';
                current_base++;
                current_start_position++;
            }
        }
        if (accum.mFileNames.size() > 0) {
            accum.setFileLabel(accum.mFileNames[0]);
            accum.mFileStartLineNumbers.resize(accum.mFileNames.size());
            // Initialize to the maximum integer value so that tests
            // will not rule that we are past a given file until the
            // actual limit is computed.
            for (unsigned i = 0; i < accum.mFileStartLineNumbers.size(); i++) {
                accum.mFileStartLineNumbers[i] = ~static_cast<size_t>(0);
            }
            f(accum.mBatchBuffer, current_start_position, &accum, mMaxCount);
        }
        alloc.deallocate(accum.mBatchBuffer, 0);
        if (accum.mLineCount > 0) grepMatchFound = true;
        return accum.mLineCount;
    }
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
        if (TraceFiles) {
            llvm::errs() << "Opened " << fileName << ".\n";
        }
        return fileDescriptor;
    }
}

// The process of searching a group of files may use a sequential or a task
// parallel approach.

void * DoGrepThreadFunction(void *args) {
    assert (args);
    return reinterpret_cast<GrepEngine *>(args)->DoGrepThreadMethod();
}

bool GrepEngine::searchAllFiles() {
    std::vector<pthread_t> threads(codegen::TaskThreads);

    for(unsigned long i = 1; i < codegen::TaskThreads; ++i) {
        const int rc = pthread_create(&threads[i], nullptr, DoGrepThreadFunction, (void *)this);
        if (rc) {
            llvm::report_fatal_error("Failed to create thread: code " + std::to_string(rc));
        }
    }
    // Main thread also does the work;
    DoGrepThreadMethod();
    for(unsigned i = 1; i < codegen::TaskThreads; ++i) {
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
    while (fileIdx < mFileGroups.size()) {
        const auto grepResult = doGrep(mFileGroups[fileIdx], mResultStrs[fileIdx]);
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
        if (pthread_self() == mEngineThread) {
            while ((mNextFileToPrint < mFileGroups.size()) && (mFileStatus[mNextFileToPrint] == FileStatus::GrepComplete)) {
                const auto output = mResultStrs[mNextFileToPrint].str();
                if (!output.empty()) {
                    llvm::outs() << output;
                }
                mFileStatus[mNextFileToPrint] = FileStatus::PrintComplete;
                mNextFileToPrint++;
            }
        }
    }
    if (pthread_self() != mEngineThread) {
        pthread_exit(nullptr);
    }
    while (mNextFileToPrint < mFileGroups.size()) {
        const bool readyToPrint = (mFileStatus[mNextFileToPrint] == FileStatus::GrepComplete);
        if (readyToPrint) {
            const auto output = mResultStrs[mNextFileToPrint].str();
            if (!output.empty()) {
                llvm::outs() << output;
            }
            mFileStatus[mNextFileToPrint] = FileStatus::PrintComplete;
            mNextFileToPrint++;
        } else {
            sched_yield();
        }
    }
    if (mGrepStdIn) {
        std::ostringstream s;
        const auto grepResult = doGrep({"-"}, s);
        llvm::outs() << s.str();
        if (grepResult) grepMatchFound = true;
    }
    return nullptr;
}

InternalSearchEngine::InternalSearchEngine(BaseDriver &driver) :
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mCaseInsensitive(false),
    mGrepDriver(driver),
    mMainMethod(nullptr),
    mNumOfThreads(1) {}

void InternalSearchEngine::grepCodeGen(re::RE * matchingRE) {
    auto & idb = mGrepDriver.getBuilder();

    re::CC * breakCC = nullptr;
    if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        breakCC = re::makeCC(0x0, &cc::Unicode);
    } else {// if (mGrepRecordBreak == GrepRecordBreakKind::LF)
        breakCC = re::makeCC(0x0A, &cc::Unicode);
    }

    matchingRE = re::exclude_CC(matchingRE, breakCC);
    matchingRE = resolveAnchors(matchingRE, breakCC);
    matchingRE = resolveCaseInsensitiveMode(matchingRE, mCaseInsensitive);
    matchingRE = regular_expression_passes(matchingRE);
    matchingRE = toUTF8(matchingRE);

    auto E = mGrepDriver.makePipeline({Binding{idb->getInt8PtrTy(), "buffer"},
                                       Binding{idb->getSizeTy(), "length"},
                                       Binding{idb->getIntAddrTy(), "accumulator"}});
    E->setNumOfThreads(mNumOfThreads);

    Scalar * const buffer = E->getInputScalar(0);
    Scalar * const length = E->getInputScalar(1);
    Scalar * const callbackObject = E->getInputScalar(2);
    StreamSet * ByteStream = E->CreateStreamSet(1, 8);
    E->CreateKernelCall<MemorySourceKernel>(buffer, length, ByteStream);

    StreamSet * RecordBreakStream = E->CreateStreamSet();
    StreamSet * BasisBits = E->CreateStreamSet(8);
    E->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    E->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{breakCC}, BasisBits, RecordBreakStream);

    StreamSet * u8index = E->CreateStreamSet();
    E->CreateKernelCall<UTF8_index>(BasisBits, u8index);

    StreamSet * MatchResults = E->CreateStreamSet();
    std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>(&cc::UTF8);
    options->setRE(matchingRE);
    options->setSource(BasisBits);
    options->setResults(MatchResults);
    options->addExternal("UTF8_index", u8index);
    E->CreateKernelCall<ICGrepKernel>(std::move(options));
    StreamSet * MatchingRecords = E->CreateStreamSet();
    E->CreateKernelCall<MatchedLinesKernel>(MatchResults, RecordBreakStream, MatchingRecords);

    if (MatchCoordinateBlocks > 0) {
        StreamSet * MatchCoords = E->CreateStreamSet(3, sizeof(size_t) * 8);
        E->CreateKernelCall<MatchCoordinatesKernel>(MatchingRecords, RecordBreakStream, MatchCoords, MatchCoordinateBlocks);
        Kernel * const matchK = E->CreateKernelCall<MatchReporter>(ByteStream, MatchCoords, callbackObject);
        matchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
        matchK->link("finalize_match_wrapper", finalize_match_wrapper);
    } else {
        Kernel * const scanMatchK = E->CreateKernelCall<ScanMatchKernel>(MatchingRecords, RecordBreakStream, ByteStream, callbackObject, ScanMatchBlocks);
        scanMatchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
        scanMatchK->link("finalize_match_wrapper", finalize_match_wrapper);
    }

    mMainMethod = E->compile();
}

InternalSearchEngine::InternalSearchEngine(const std::unique_ptr<grep::GrepEngine> & engine)
    : InternalSearchEngine(engine->mGrepDriver) {}

InternalSearchEngine::~InternalSearchEngine() { }


void InternalSearchEngine::doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum) {
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length, MatchAccumulator *);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);
    f(search_buffer, bufferLength, &accum);
}

class LineNumberAccumulator : public grep::MatchAccumulator {
public:
    LineNumberAccumulator() {}
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    std::vector<uint64_t> && getAccumulatedLines() { return std::move(mLineNums); }
private:
    std::vector<uint64_t> mLineNums;
};

void LineNumberAccumulator::accumulate_match(const size_t lineNum, char * /* line_start */, char * /* line_end */) {
    mLineNums.push_back(lineNum);
}

std::vector<uint64_t> lineNumGrep(re::RE * pattern, const char * buffer, size_t bufSize) {
      LineNumberAccumulator accum;
      CPUDriver driver("driver");
      grep::InternalSearchEngine engine(driver);
      engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
      engine.grepCodeGen(pattern);
      engine.doGrep(buffer, bufSize, accum);
      return accum.getAccumulatedLines();
  }

InternalMultiSearchEngine::InternalMultiSearchEngine(BaseDriver &driver) :
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mCaseInsensitive(false),
    mGrepDriver(driver),
    mMainMethod(nullptr),
    mNumOfThreads(1) {}

void InternalMultiSearchEngine::grepCodeGen(const re::PatternVector & patterns) {
    auto & idb = mGrepDriver.getBuilder();

    re::CC * breakCC = nullptr;
    if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
        breakCC = re::makeByte(0x0);
    } else {// if (mGrepRecordBreak == GrepRecordBreakKind::LF)
        breakCC = re::makeByte(0x0A);
    }

    auto E = mGrepDriver.makePipeline({Binding{idb->getInt8PtrTy(), "buffer"},
        Binding{idb->getSizeTy(), "length"},
        Binding{idb->getIntAddrTy(), "accumulator"}});
    E->setNumOfThreads(mNumOfThreads);

    Scalar * const buffer = E->getInputScalar(0);
    Scalar * const length = E->getInputScalar(1);
    Scalar * const callbackObject = E->getInputScalar(2);
    StreamSet * ByteStream = E->CreateStreamSet(1, 8);
    E->CreateKernelCall<MemorySourceKernel>(buffer, length, ByteStream);

    StreamSet * RecordBreakStream = E->CreateStreamSet();
    StreamSet * BasisBits = E->CreateStreamSet(8);
    E->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    E->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{breakCC}, BasisBits, RecordBreakStream);

    StreamSet * u8index = E->CreateStreamSet();
    E->CreateKernelCall<UTF8_index>(BasisBits, u8index);

    StreamSet * resultsSoFar = RecordBreakStream;

    const auto n = patterns.size();

    for (unsigned i = 0; i < n; i++) {
        StreamSet * const MatchResults = E->CreateStreamSet();

        auto options = make_unique<GrepKernelOptions>();

        auto r = resolveCaseInsensitiveMode(patterns[i].second, mCaseInsensitive);
        r = re::exclude_CC(r, breakCC);
        r = resolveAnchors(r, breakCC);
        r = regular_expression_passes(r);
        r = toUTF8(r);

        options->setRE(r);
        options->setSource(BasisBits);
        options->setResults(MatchResults);
        const auto isExclude = patterns[i].first == re::PatternKind::Exclude;
        if (i != 0 || !isExclude) {
            options->setCombiningStream(isExclude ? GrepCombiningType::Exclude : GrepCombiningType::Include, resultsSoFar);
        }
        options->addExternal("UTF8_index", u8index);
        E->CreateKernelCall<ICGrepKernel>(std::move(options));
        resultsSoFar = MatchResults;
    }

    if (MatchCoordinateBlocks > 0) {
        StreamSet * MatchCoords = E->CreateStreamSet(3, sizeof(size_t) * 8);
        E->CreateKernelCall<MatchCoordinatesKernel>(resultsSoFar, RecordBreakStream, MatchCoords, MatchCoordinateBlocks);
        Kernel * const matchK = E->CreateKernelCall<MatchReporter>(ByteStream, MatchCoords, callbackObject);
        matchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
        matchK->link("finalize_match_wrapper", finalize_match_wrapper);
    } else {
        Kernel * const scanMatchK = E->CreateKernelCall<ScanMatchKernel>(resultsSoFar, RecordBreakStream, ByteStream, callbackObject, ScanMatchBlocks);
        scanMatchK->link("accumulate_match_wrapper", accumulate_match_wrapper);
        scanMatchK->link("finalize_match_wrapper", finalize_match_wrapper);
    }

    mMainMethod = E->compile();
}

void InternalMultiSearchEngine::doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum) {
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length, MatchAccumulator *);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);
    f(search_buffer, bufferLength, &accum);
}

InternalMultiSearchEngine::InternalMultiSearchEngine(const std::unique_ptr<grep::GrepEngine> & engine)
: InternalMultiSearchEngine(engine->mGrepDriver) {

}

InternalMultiSearchEngine::~InternalMultiSearchEngine() { }

}
