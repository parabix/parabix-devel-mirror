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
#include <kernels/callback.h>
#include <kernels/charclasses.h>
#include <cc/cc_kernel.h>
#include <grep/grep_kernel.h>
#include <kernels/UCD_property_kernel.h>
#include <kernels/grapheme_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/core/streamset.h>
#include <kernels/until_n.h>
#include <kernels/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <cc/alphabet.h>
#include <re/re_cc.h>
#include <re/re_diff.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_alt.h>
#include <re/re_end.h>
#include <re/re_name.h>
#include <re/re_assertion.h>
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
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Casting.h>
#include <kernels/pipeline_builder.h>
#include <sched.h>
#include <atomic>

using namespace llvm;
using namespace cc;
using namespace kernel;
namespace grep {

static cl::opt<int> Threads("t", cl::desc("Total number of threads."), cl::init(2));
static cl::opt<bool> PabloTransposition("enable-pablo-s2p", cl::desc("Enable experimental pablo transposition."));
static cl::opt<bool> CC_Multiplexing("CC-multiplexing", cl::desc("Enable CC multiplexing."), cl::init(false));
static cl::opt<bool> PropertyKernels("enable-property-kernels", cl::desc("Enable Unicode property kernels."), cl::init(true));
static cl::opt<bool> MultithreadedSimpleRE("enable-simple-RE-kernels", cl::desc("Enable individual CC kernels for simple REs."), cl::init(false));
static cl::opt<int> MatchCoordinateBlocks("match-coordinates", cl::desc("Enable experimental MatchCoordinates kernels with a given number of blocks per stride"), cl::init(1));
const unsigned DefaultByteCClimit = 6;

unsigned ByteCClimit;
static cl::opt<unsigned, true> ByteCClimitOption("byte-CC-limit", cl::location(ByteCClimit), cl::desc("Max number of CCs for byte CC pipeline."), cl::init(DefaultByteCClimit));

const auto ENCODING_BITS = 8;

void GrepCallBackObject::handle_signal(unsigned s) {
    if (static_cast<GrepSignal>(s) == GrepSignal::BinaryFile) {
        mBinaryFile = true;
    } else {
        llvm::report_fatal_error("Unknown GrepSignal");
    }
}

const unsigned CoordsPerMatch = 3;
enum MatchCoordinatesEnum {LINE_STARTS = 0, LINE_ENDS = 1, LINE_NUMBERS = 2};
#define line_start(coordIdx)  coords[coordIdx + LINE_STARTS]
#define line_end(coordIdx)  coords[coordIdx + LINE_ENDS]
#define line_num(coordIdx)  coords[coordIdx + LINE_NUMBERS]
#define line_num1(coordIdx)  coords[coordIdx + LINE_NUMBERS] + 1
#define line_length(coordIdx)  coords[coordIdx + LINE_ENDS] - coords[coordIdx + LINE_STARTS] + 1

void MatchAccumulator::report_matches(char * buffer_base, size_t coordsCount, size_t * coords) {
    for (unsigned i = 0; i < coordsCount; i+=CoordsPerMatch) {
        accumulate_match(line_num(i), &buffer_base[line_start(i)], &buffer_base[line_end(i)]);
    }
}

extern "C" void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, char * line_start, char * line_end) {
    assert ("passed a null accumulator" && accum_addr);
    reinterpret_cast<MatchAccumulator *>(accum_addr)->accumulate_match(lineNum, line_start, line_end);
}

extern "C" void report_matches_wrapper(intptr_t accum_addr, char * buffer_base, size_t coordsCount, size_t * coords) {
    reinterpret_cast<MatchAccumulator *>(accum_addr)->report_matches(buffer_base, coordsCount, coords);
}

extern "C" void finalize_match_wrapper(intptr_t accum_addr, char * buffer_end) {
    assert ("passed a null accumulator" && accum_addr);
    reinterpret_cast<MatchAccumulator *>(accum_addr)->finalize_match(buffer_end);
}

// Grep Engine construction and initialization.

GrepEngine::GrepEngine(BaseDriver &driver) :
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
    mGrepDriver(driver),
    mMainMethod(nullptr),
    mNextFileToGrep(0),
    mNextFileToPrint(0),
    grepMatchFound(false),
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mRequiredComponents(static_cast<Component>(0)),
    mMoveMatchesToEOL(true),
    mEngineThread(pthread_self()) {}

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

void GrepEngine::initFileResult(const std::vector<boost::filesystem::path> & paths) {
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
    if ((mEngineKind == EngineKind::EmitMatches) || (mEngineKind == EngineKind::CountOnly)) {
        if (!allAnchored || (mGrepRecordBreak == GrepRecordBreakKind::Unicode)) {
            setComponent(mRequiredComponents, Component::MoveMatchesToEOL);
        }
    }
}

// Code Generation
//
// All engines share a common pipeline to compute a stream of Matches from a given input Bytestream.

std::pair<StreamSet *, StreamSet *> GrepEngine::grepPipeline(const std::unique_ptr<ProgramBuilder> & P, StreamSet * InputStream) {

    Scalar * const callbackObject = P->getInputScalar("callbackObject");

    //  Regular Expression Processing and Analysis Phase

    StreamSet * ByteStream = nullptr;
    if (mBinaryFilesMode == argv::Text) {
        ByteStream = InputStream;
    } else if (mBinaryFilesMode == argv::WithoutMatch) {
        ByteStream = P->CreateStreamSet(1, 8);
        Kernel * binaryCheckK = P->CreateKernelCall<AbortOnNull>(InputStream, ByteStream, callbackObject);
        mGrepDriver.LinkFunction(binaryCheckK, "signal_dispatcher", kernel::signal_dispatcher);
    } else {
        llvm::report_fatal_error("Binary mode not supported.");
    }

    const auto numOfREs = mREs.size();
    SmallVector<bool, 4> hasGCB(numOfREs);
    bool anyGCB = false;

    for(unsigned i = 0; i < numOfREs; ++i) {
        hasGCB[i] = hasGraphemeClusterBoundary(mREs[i]);
        anyGCB |= hasGCB[i];
    }
    if (anyGCB) {
        setComponent(mRequiredComponents, Component::GraphemeClusterBoundary);
    }

    StreamSet * LineBreakStream = P->CreateStreamSet();
    std::vector<StreamSet *> MatchResultsBufs(numOfREs);

    re::RE * prefixRE;
    re::RE * suffixRE;

    // For simple regular expressions with a small number of characters, we
    // can bypass transposition and use the Direct CC compiler.
    const auto isSimple = (numOfREs == 1) && (mGrepRecordBreak != GrepRecordBreakKind::Unicode) && (!anyGCB) && mUnicodeProperties.empty();
    const auto isWithinByteTestLimit = byteTestsWithinLimit(mREs[0], ByteCClimit);
    const auto hasTriCC = hasTriCCwithinLimit(mREs[0], ByteCClimit, prefixRE, suffixRE);
    const auto internalS2P = isSimple && (isWithinByteTestLimit || hasTriCC);

    Component internalComponents = Component::NoComponents;
    if (internalS2P && hasComponent(mRequiredComponents, Component::MoveMatchesToEOL)) {
        setComponent(internalComponents, Component::MoveMatchesToEOL);
    }

    StreamSet * SourceStream = ByteStream;
    StreamSet * const U8index = P->CreateStreamSet();
    StreamSet * UnicodeLB = nullptr;
    std::map<std::string, StreamSet *> propertyStream;
    StreamSet * GCB_stream = nullptr;

    if (!internalS2P) {
        StreamSet * BasisBits = P->CreateStreamSet(ENCODING_BITS, 1);
        if (PabloTransposition) {
            P->CreateKernelCall<S2P_PabloKernel>(SourceStream, BasisBits);
        } else {
            //P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
            Kernel * s2pK = P->CreateKernelCall<S2PKernel>(SourceStream, BasisBits, callbackObject);
            mGrepDriver.LinkFunction(s2pK, "signal_dispatcher", kernel::signal_dispatcher);
        }
        SourceStream = BasisBits;
    }

    if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
        UnicodeLB = P->CreateStreamSet();
        StreamSet * const LineFeedStream = P->CreateStreamSet();
        P->CreateKernelCall<LineFeedKernelBuilder>(SourceStream, LineFeedStream);
        P->CreateKernelCall<RequiredStreams_UTF8>(SourceStream, LineFeedStream, U8index, UnicodeLB);
        LineBreakStream = UnicodeLB;
    }
    else if (!internalS2P) {
        P->CreateKernelCall<UTF8_index>(SourceStream, U8index);
        if (mGrepRecordBreak == GrepRecordBreakKind::LF) {
            P->CreateKernelCall<LineFeedKernelBuilder>(SourceStream, LineBreakStream);
        } else { // if (mGrepRecordBreak == GrepRecordBreakKind::Null) {
            P->CreateKernelCall<CharacterClassKernelBuilder>( "Null", std::vector<re::CC *>{mBreakCC}, SourceStream, LineBreakStream);
        }
    }

    if (PropertyKernels) {
        for (auto p : mUnicodeProperties) {
            auto name = p->getFullName();
            StreamSet * property = P->CreateStreamSet(1, 1);
            propertyStream.emplace(name, property);
            P->CreateKernelCall<UnicodePropertyKernelBuilder>(p, SourceStream, property);
        }
    }

    if (hasComponent(mRequiredComponents, Component::GraphemeClusterBoundary)) {
        GCB_stream = P->CreateStreamSet();
        P->CreateKernelCall<GraphemeClusterBreakKernel>(SourceStream, U8index, GCB_stream);
    }

    for(unsigned i = 0; i < numOfREs; ++i) {
        StreamSet * const MatchResults = P->CreateStreamSet(1, 1);
        MatchResultsBufs[0] = MatchResults;
        std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
        options->setIndexingAlphabet(&cc::UTF8);
        options->setSource(SourceStream);
        options->setResults(MatchResults);
        if (hasComponent(internalComponents, Component::MoveMatchesToEOL)) {
            re::RE * notBreak = re::makeDiff(re::makeByte(0x00, 0xFF), mBreakCC);
            if (isWithinByteTestLimit) {
                mREs[i] = re::makeSeq({mREs[i], re::makeRep(notBreak, 0, re::Rep::UNBOUNDED_REP), makeNegativeLookAheadAssertion(notBreak)});
            } else {
                suffixRE = re::makeSeq({suffixRE, re::makeRep(notBreak, 0, re::Rep::UNBOUNDED_REP), makeNegativeLookAheadAssertion(notBreak)});
            }
        }
        options->setRE(mREs[i]);
        std::set<re::Name *> props;
        re::gatherUnicodeProperties(mREs[i], props);
        for (const auto & p : props) {
            auto name = p->getFullName();
            const auto f = propertyStream.find(name);
            if (f != propertyStream.end()) {
                options->addExternal(name, f->second);
            }
        }
        if (hasGCB[i]) { assert (GCB_stream);
            options->addExternal("\\b{g}", GCB_stream);
        }
        if (internalS2P) {
            if (!isWithinByteTestLimit) {
                if (MultithreadedSimpleRE) {
                    auto CCs = re::collectCCs(prefixRE, cc::Byte);
                    for (auto cc : CCs) {
                        auto ccName = makeName(cc);
                        prefixRE = re::replaceCC(prefixRE, cc, ccName);
                        suffixRE = re::replaceCC(suffixRE, cc, ccName);
                        auto ccNameStr = ccName->getFullName();
                        StreamSet * const ccStream = P->CreateStreamSet(1, 1);
                        P->CreateKernelCall<CharacterClassKernelBuilder>(ccNameStr, std::vector<re::CC *>{cc}, SourceStream, ccStream);
                        options->addExternal(ccNameStr, ccStream);
                    }
                }
                options->setPrefixRE(prefixRE);
                options->setRE(suffixRE);
            }
            Kernel * LB_nullK = P->CreateKernelCall<CharacterClassKernelBuilder>( "breakCC", std::vector<re::CC *>{mBreakCC}, SourceStream, LineBreakStream, callbackObject);
            mGrepDriver.LinkFunction(LB_nullK, "signal_dispatcher", kernel::signal_dispatcher);
        } else {
            options->addExternal("UTF8_index", U8index);
            if (mGrepRecordBreak == GrepRecordBreakKind::Unicode) {
                options->addExternal("UTF8_LB", LineBreakStream);
            }
            if (CC_Multiplexing) {
                const auto UnicodeSets = re::collectCCs(mREs[i], cc::Unicode, std::set<re::Name *>{re::makeZeroWidth("\\b{g}")});
                if (UnicodeSets.size() <= 1) {
                    options->setRE(mREs[i]);
                } else {
                    auto mpx = std::make_shared<MultiplexedAlphabet>("mpx", UnicodeSets);
                    mREs[i] = transformCCs(mpx, mREs[i]);
                    options->setRE(mREs[i]);
                    auto mpx_basis = mpx->getMultiplexedCCs();
                    StreamSet * const CharClasses = P->CreateStreamSet(mpx_basis.size());
                    P->CreateKernelCall<CharClassesKernel>(std::move(mpx_basis), SourceStream, CharClasses);
                    options->addAlphabet(mpx, CharClasses);
                }
            }
        }
        P->CreateKernelCall<ICGrepKernel>(std::move(options));
    }

    StreamSet * Matches = MatchResultsBufs[0];
    if (MatchResultsBufs.size() > 1) {
        StreamSet * const MergedMatches = P->CreateStreamSet();
        P->CreateKernelCall<StreamsMerge>(MatchResultsBufs, MergedMatches);
        Matches = MergedMatches;
    }
    if (hasComponent(mRequiredComponents, Component::MoveMatchesToEOL) && !hasComponent(internalComponents, Component::MoveMatchesToEOL)) {
        StreamSet * const MovedMatches = P->CreateStreamSet();
        P->CreateKernelCall<MatchedLinesKernel>(Matches, LineBreakStream, MovedMatches);
        Matches = MovedMatches;
    }
    if (mInvertMatches) {
        StreamSet * const InvertedMatches = P->CreateStreamSet();
        P->CreateKernelCall<InvertMatchesKernel>(Matches, LineBreakStream, InvertedMatches);
        Matches = InvertedMatches;
    }
    if (mMaxCount > 0) {
        StreamSet * const TruncatedMatches = P->CreateStreamSet();
        Scalar * const maxCount = P->getInputScalar("maxCount");
        P->CreateKernelCall<UntilNkernel>(maxCount, Matches, TruncatedMatches);
        Matches = TruncatedMatches;
    }
    return std::pair<StreamSet *, StreamSet *>(LineBreakStream, Matches);
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
    StreamSet * const Matches = grepPipeline(P, ByteStream).second;
    P->CreateKernelCall<PopcountKernel>(Matches, P->getOutputScalar("countResult"));

    mMainMethod = P->compile();
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

    const auto bytes = line_end - line_start + 1;
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
    
void EmitMatch::report_matches(char * buffer_base, size_t coordsCount, size_t * coords) {
    if (coordsCount == 0) return;
    std::string colonSep = ":";
    if (mInitialTab) colonSep = "\t:";
    if (mShowLineNumbers && mShowByteNumbers) {
        for (unsigned i = 0; i < coordsCount; i+=CoordsPerMatch) {
            mResultStr << mLinePrefix << line_num1(i) << colonSep << line_start(i) << colonSep;
            mResultStr.write(&buffer_base[line_start(i)], line_length(i));
            mLineCount++;
        }
    } else if (mShowLineNumbers) {
        for (unsigned i = 0; i < coordsCount; i+=CoordsPerMatch) {
            mResultStr << mLinePrefix << line_num1(i) << colonSep;
            mResultStr.write(&buffer_base[line_start(i)], line_length(i));
            mLineCount++;
        }
    } else if (mShowByteNumbers) {
        for (unsigned i = 0; i < coordsCount; i+=CoordsPerMatch) {
            mResultStr << mLinePrefix << line_start(i) << colonSep;
            mResultStr.write(&buffer_base[line_start(i)], line_length(i));
            mLineCount++;
       }
    } else {
        for (unsigned i = 0; i < coordsCount; i+=CoordsPerMatch) {
            mResultStr << mLinePrefix;
            mResultStr.write(&buffer_base[line_start(i)], line_length(i));
            mLineCount++;
       }
    }
    unsigned last_byte_pos = line_end(coordsCount-3);
    unsigned last_byte = buffer_base[last_byte_pos];
    mTerminated = (last_byte >= 0x0A) && (last_byte <= 0x0D);
    if (LLVM_UNLIKELY(!mTerminated)) {
        if (last_byte == 0x85) {  //  Possible NEL terminator.
            mTerminated = (last_byte_pos >= 1) && (static_cast<unsigned>(buffer_base[last_byte_pos-1]) == 0xC2);
        }
        else {
            // Possible LS or PS terminators.
            mTerminated = (last_byte_pos >= 2) && (static_cast<unsigned>(buffer_base[last_byte_pos-2]) == 0xE2)
                                       && (static_cast<unsigned>(buffer_base[last_byte_pos-1]) == 0x80)
                                       && ((last_byte == 0xA8) || (last_byte == 0xA9));
        }
    }
}


void EmitMatch::finalize_match(char * buffer_end) {
    if (!mTerminated) mResultStr << "\n";
}

void EmitMatchesEngine::grepCodeGen() {
    auto & idb = mGrepDriver.getBuilder();

    auto E = mGrepDriver.makePipeline(
                // inputs
                {Binding{idb->getSizeTy(), "useMMap"},
                Binding{idb->getInt32Ty(), "fileDescriptor"},
                Binding{idb->getIntAddrTy(), "callbackObject"},
                Binding{idb->getSizeTy(), "maxCount"}}
                ,// output
                {Binding{idb->getInt64Ty(), "countResult"}});

    Scalar * const useMMap = E->getInputScalar("useMMap");
    Scalar * const fileDescriptor = E->getInputScalar("fileDescriptor");

    StreamSet * const ByteStream = E->CreateStreamSet(1, ENCODING_BITS);
    E->CreateKernelCall<FDSourceKernel>(useMMap, fileDescriptor, ByteStream);

    StreamSet * LineBreakStream;
    StreamSet * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(E, ByteStream);

    if (MatchCoordinateBlocks > 0) {
        StreamSet * MatchCoords = E->CreateStreamSet(1, sizeof(size_t) * 8);
        E->CreateKernelCall<MatchCoordinatesKernel>(Matches, LineBreakStream, MatchCoords, MatchCoordinateBlocks);
        Scalar * const callbackObject = E->getInputScalar("callbackObject");
        Kernel * const matchK = E->CreateKernelCall<MatchReporter>(ByteStream, MatchCoords, callbackObject);
        mGrepDriver.LinkFunction(matchK, "report_matches_wrapper", report_matches_wrapper);
        mGrepDriver.LinkFunction(matchK, "finalize_match_wrapper", finalize_match_wrapper);
    } else {
        Scalar * const callbackObject = E->getInputScalar("callbackObject");
        Kernel * const scanMatchK = E->CreateKernelCall<ScanMatchKernel>(Matches, LineBreakStream, ByteStream, callbackObject);
        mGrepDriver.LinkFunction(scanMatchK, "accumulate_match_wrapper", accumulate_match_wrapper);
        mGrepDriver.LinkFunction(scanMatchK, "finalize_match_wrapper", finalize_match_wrapper);
    }
    E->setOutputScalar("countResult", E->CreateConstant(idb->getInt64(0)));

    mMainMethod = E->compile();
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
    typedef uint64_t (*GrepFunctionType)(bool useMMap, int32_t fileDescriptor, GrepCallBackObject *, size_t maxCount);
    bool useMMap = mPreferMMap && canMMap(fileName);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);

    int32_t fileDescriptor = openFile(fileName, strm);
    if (fileDescriptor == -1) return 0;
    GrepCallBackObject handler;
    uint64_t grepResult = f(useMMap, fileDescriptor, &handler, mMaxCount);

    close(fileDescriptor);
    if (handler.binaryFileSignalled()) {
        llvm::errs() << "Binary file " << fileName << "\n";
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
    typedef uint64_t (*GrepFunctionType)(bool useMMap, int32_t fileDescriptor, EmitMatch *, size_t maxCount);
    bool useMMap = mPreferMMap && canMMap(fileName);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);
    int32_t fileDescriptor = openFile(fileName, strm);
    if (fileDescriptor == -1) return 0;
    EmitMatch accum(linePrefix(fileName), mShowLineNumbers, mInitialTab, strm);
    f(useMMap, fileDescriptor, &accum, mMaxCount);
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
    assert (args);
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
        if (pthread_self() == mEngineThread) {
            while ((mNextFileToPrint < inputPaths.size()) && (mFileStatus[mNextFileToPrint] == FileStatus::GrepComplete)) {
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
    while (mNextFileToPrint < inputPaths.size()) {
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
        const auto grepResult = doGrep("-", s);
        llvm::outs() << s.str();
        if (grepResult) grepMatchFound = true;
    }
    return nullptr;
}

InternalSearchEngine::InternalSearchEngine(BaseDriver &driver) :
    mGrepRecordBreak(GrepRecordBreakKind::LF),
    mCaseInsensitive(false),
    mGrepDriver(driver),
    mMainMethod(nullptr) {}

void InternalSearchEngine::grepCodeGen(re::RE * matchingRE, re::RE * excludedRE) {
    auto & idb = mGrepDriver.getBuilder();
    mSaveSegmentPipelineParallel = codegen::SegmentPipelineParallel;
    codegen::SegmentPipelineParallel = false;

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

    auto E = mGrepDriver.makePipeline({Binding{idb->getInt8PtrTy(), "buffer"},
                                       Binding{idb->getSizeTy(), "length"},
                                       Binding{idb->getIntAddrTy(), "accumulator"}});

    Scalar * const buffer = E->getInputScalar(0);
    Scalar * const length = E->getInputScalar(1);
    StreamSet * ByteStream = E->CreateStreamSet(1, 8);
    E->CreateKernelCall<MemorySourceKernel>(buffer, length, ByteStream);


    StreamSet * RecordBreakStream = E->CreateStreamSet();
    const auto RBname = (mGrepRecordBreak == GrepRecordBreakKind::Null) ? "Null" : "LF";

    StreamSet * BasisBits = nullptr;

    if (matchAllLines && excludeNothing) {
        E->CreateKernelCall<CharacterClassKernelBuilder>(RBname, std::vector<re::CC *>{breakCC}, ByteStream, RecordBreakStream);
    } else {
        BasisBits = E->CreateStreamSet(8);
        E->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
        E->CreateKernelCall<CharacterClassKernelBuilder>(RBname, std::vector<re::CC *>{breakCC}, BasisBits, RecordBreakStream);
    }

    StreamSet * MatchingRecords = nullptr;
    if (matchAllLines) {
        MatchingRecords = RecordBreakStream;
    } else {
        StreamSet * MatchResults = E->CreateStreamSet();
        std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
        options->setRE(matchingRE);
        options->setSource(BasisBits);
        options->setResults(MatchResults);
        E->CreateKernelCall<ICGrepKernel>(std::move(options));
        MatchingRecords = E->CreateStreamSet();
        E->CreateKernelCall<MatchedLinesKernel>(MatchResults, RecordBreakStream, MatchingRecords);
    }
    if (!excludeNothing) {
        StreamSet * ExcludedResults = E->CreateStreamSet();
        std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
        options->setRE(excludedRE);
        options->setSource(BasisBits);
        options->setResults(ExcludedResults);
        E->CreateKernelCall<ICGrepKernel>(std::move(options));
        StreamSet * ExcludedRecords = E->CreateStreamSet();
        E->CreateKernelCall<MatchedLinesKernel>(ExcludedResults, RecordBreakStream, ExcludedRecords);

        if (!matchAllLines) {
            StreamSet * nonExcluded = E->CreateStreamSet();
            E->CreateKernelCall<InvertMatchesKernel>(ExcludedRecords, RecordBreakStream, nonExcluded);
            StreamSet * const included = MatchingRecords;
            MatchingRecords = E->CreateStreamSet();
            E->CreateKernelCall<StreamsIntersect>(std::vector<StreamSet *>{included, nonExcluded}, MatchingRecords);
        } else {
            MatchingRecords = E->CreateStreamSet();
            E->CreateKernelCall<InvertMatchesKernel>(ExcludedRecords, RecordBreakStream, MatchingRecords);
        }
    }

    Kernel * scanMatchK = E->CreateKernelCall<ScanMatchKernel>(MatchingRecords, RecordBreakStream, ByteStream, E->getInputScalar(2));
    mGrepDriver.LinkFunction(scanMatchK, "accumulate_match_wrapper", accumulate_match_wrapper);
    mGrepDriver.LinkFunction(scanMatchK, "finalize_match_wrapper", finalize_match_wrapper);

    mMainMethod = E->compile();
}

void InternalSearchEngine::doGrep(const char * search_buffer, size_t bufferLength, MatchAccumulator & accum) {
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length, MatchAccumulator *);
    auto f = reinterpret_cast<GrepFunctionType>(mMainMethod);
    f(search_buffer, bufferLength, &accum);
    codegen::SegmentPipelineParallel = mSaveSegmentPipelineParallel;
}

GrepEngine::~GrepEngine() { }

InternalSearchEngine::InternalSearchEngine(const std::unique_ptr<grep::GrepEngine> & engine)
: InternalSearchEngine(engine->mGrepDriver) {

}

InternalSearchEngine::~InternalSearchEngine() { }

}
