/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "post_process.h"
#include "test_suite_error.h"
#include "xml_linebreak_kernel.hpp"

#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/scan/scan.h>
#include <kernel/streamutils/collapse.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/streamutils/streams_merge.h>
#include <kernel/streamutils/multiplex.h>
#include <kernel/util/linebreak_kernel.h>
#include <kernel/util/debug_display.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Path.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/parser.h>
#include <pablo/parse/lexer.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <toolchain/toolchain.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sys/stat.h>

namespace fs = boost::filesystem;
namespace su = kernel::streamutils;

using namespace llvm;
using namespace kernel;
using namespace pablo;
using namespace pablo::parse;


static cl::OptionCategory xmlFlags("Command Flags", "xml options");

std::string inputFile;
static cl::opt<std::string, true> inputFileOption(cl::Positional, cl::location(inputFile), cl::desc("<input file>"), cl::Required, cl::cat(xmlFlags));

typedef void(*XMLProcessFunctionType)(uint32_t fd);

XMLProcessFunctionType xmlPipelineGen(CPUDriver & pxDriver, std::shared_ptr<PabloParser> parser, std::shared_ptr<SourceFile> xmlPabloSrc) {
    const size_t ERROR_STREAM_COUNT = 9;
    auto & iBuilder = pxDriver.getBuilder();
    Type * const i32Ty = iBuilder->getInt32Ty();
    auto P = pxDriver.makePipeline({Binding{i32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    StreamSet * const BasisBits = P->CreateStreamSet(8, 1);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    StreamSet * const Lex = P->CreateStreamSet(27, 1);
    StreamSet * const U8 = P->CreateStreamSet(1, 1);
    StreamSet * const LexError = P->CreateStreamSet(ERROR_STREAM_COUNT, 1);
    P->CreateKernelCall<PabloSourceKernel>(
        parser,
        xmlPabloSrc,
        "ClassifyBytesValidateUtf8",
        Bindings { // Input Stream Bindings
            Binding {"basis", BasisBits} 
        }, 
        Bindings { // Output Stream Bindings
            Binding {"lex", Lex}, 
            Binding {"u8", U8}, 
            Binding {"err", LexError, FixedRate(1), Add1()} 
        }
    );

    StreamSet * const Marker = P->CreateStreamSet(5, 1);
    StreamSet * const CCPCallouts = P->CreateStreamSet(9, 1);
    StreamSet * const CCPError = P->CreateStreamSet(ERROR_STREAM_COUNT, 1);
    P->CreateKernelCall<PabloSourceKernel>(
        parser,
        xmlPabloSrc,
        "Preprocess",
        Bindings { // Input Stream Bindings
            Binding {"lex", Lex}
        },
        Bindings { // Output Stream Bindings
            Binding {"marker", Marker},
            Binding {"callouts", CCPCallouts},
            Binding {"err", CCPError, FixedRate(1), Add1()}
        }
    );

    StreamSet * const TagError = P->CreateStreamSet(ERROR_STREAM_COUNT, 1);
    StreamSet * const TagCallouts = P->CreateStreamSet(10, 1);
    StreamSet * const PostProcessTagMatchingStreams = P->CreateStreamSet(5, 1);
    P->CreateKernelCall<PabloSourceKernel>(
        parser,
        xmlPabloSrc,
        "ParseTags",
        Bindings { // Input Stream Bindings
            Binding {"lex", Lex},
            Binding {"marker", Marker}
        },
        Bindings { // Output Stream Bindings
            Binding {"callouts", TagCallouts},
            Binding {"post", PostProcessTagMatchingStreams},
            Binding {"err", TagError, FixedRate(1), Add1()}
        }
    );

    StreamSet * const RefError = P->CreateStreamSet(ERROR_STREAM_COUNT, 1);
    StreamSet * const RefCallouts = P->CreateStreamSet(6, 1);
    P->CreateKernelCall<PabloSourceKernel>(
        parser,
        xmlPabloSrc,
        "ParseRef",
        Bindings { // Input Stream Bindings
            Binding {"lex", Lex},
            Binding {"marker", Marker}
        },
        Bindings { // Output Stream Bindings
            Binding {"callouts", RefCallouts},
            Binding {"err", RefError, FixedRate(1), Add1()}
        }
    );

    StreamSet * const CheckStreams = P->CreateStreamSet(3 ,1);
    StreamSet * const NameError = P->CreateStreamSet(ERROR_STREAM_COUNT, 1);
    P->CreateKernelCall<PabloSourceKernel>(
        parser,
        xmlPabloSrc,
        "ValidateXmlNames",
        Bindings { // Input Stream Bindings
            Binding {"ccp", CCPCallouts},
            Binding {"tag", TagCallouts},
            Binding {"ref", RefCallouts},
            Binding {"marker", Marker},
            Binding {"lex", Lex},
            Binding {"u8", U8}
        },
        Bindings { // Output Stream Bindings
            Binding {"check", CheckStreams},
            Binding {"err", NameError, FixedRate(1), Add1()}
        }
    );

    StreamSet * const Errors = P->CreateStreamSet(ERROR_STREAM_COUNT, 1);
    P->CreateKernelCall<StreamsMerge>(
        std::vector<StreamSet *>{LexError, CCPError, TagError, RefError, NameError}, 
        Errors
    );

    /* Post Process Scanning */

    /*
        Tag Matching

        A stream of start and end element names is scanned through with each
        start name being pushed onto a stack. When a end name is encountered,
        it is compared to the top name in the stack. If they don't match, a
        tag name missmatch error is logged.

        The ending "/>" of empty tags (e.g., `<node/>`) treated as an end tag
        name. The post processing algorithm will recognise this case and pop
        the top name (which will be the name of the empty tag) off the stack.
     */
    { // Tag Matching Scope
        StreamSet * const StartIndices = scan::ToIndices(P, su::Select(P, PostProcessTagMatchingStreams, 0));
        StreamSet * const EndIndices = scan::ToIndices(P, su::Select(P, PostProcessTagMatchingStreams, 1));
        StreamSet * const BasisStreams = su::Select(P, PostProcessTagMatchingStreams, su::Range(2, 5));
        StreamSet * const Codes = su::Multiplex(P, BasisStreams);
        scan::Reader(P, pxDriver,
            SCAN_CALLBACK(postproc_tagMatcher),
            ByteStream,
            { StartIndices, EndIndices },
            { StartIndices, Codes });
    } // End Tag Matching Scope


    /*
        Duplicate Attribute Detection

        For each element, attribute names will be sequentially added to a set
        to ensure that there are no duplicate attributes. At the end of an
        element's attribute list, the set will be cleared out so that it is
        ready to validate the next element.

        Example:
            <node attr="..." attr="..."/>
                  ^   ^      ^   ^     ^
                  a   b      c   d     x

            Scan Reader Input (processed from top to bottom):
                start   end   codes
                -----  -----  -----
                  a      b     0x1: attr name
                  c      d     0x1: attr name
                  x      x     0x2: list end
     */
    { // Duplicate Attribute Detection Scope
        StreamSet * const BasisStreams = su::Select(P, TagCallouts, {2, 9});

        // `StartIndices` are the primary scanning indices.
        //
        // Consists of the following streams merged together:
        //  - attrNameStarts
        //  - attrListEnds
        //
        // `attrListStarts` always has a bit at the start of the first attribute
        // name meaning we don't need to include it in this merge. The start of
        // a new list is determined by the multiplexed code.
        StreamSet * const StartIndices = P->CreateStreamSet(1, 64);
        P->CreateKernelCall<ScanIndexGenerator>(su::Merge(P, TagCallouts, {2, 9}), StartIndices);

        // `EndIndices` are an additional stream which denotes the ends of
        // attribute names or values.
        //
        // Consists of the following streams merged together:
        //  - attrNameEnds
        //  - attrListEnds
        //
        // `attrListEnds` is present in both start and end index streams as there
        // needs to be a one-to-one correspondence between the two streams.
        StreamSet * const EndIndices = P->CreateStreamSet(1, 64);
        P->CreateKernelCall<ScanIndexGenerator>(su::Merge(P, TagCallouts, {3, 9}), EndIndices);

        StreamSet * const Codes = su::Multiplex(P, BasisStreams);
        scan::Reader(P, pxDriver, 
            SCAN_CALLBACK(postproc_duplicateAttrDetector), 
            ByteStream, 
            { StartIndices, EndIndices },
            { StartIndices, Codes });
    } // End Duplication Attribute Detection Scope


    // Use a custom linebreak kernel which always places a bit at EOF even if
    // the file ends in a linebreak. This is done to simplify the LineSpan
    // logic while maintaining the ability to find errors which occur at EOF.
    StreamSet * const LineBreakStream = XmlLineBreaks(P, ByteStream);
    StreamSet * const LineSpans = scan::LineSpans(P, LineBreakStream);


    /**
     * Post processing for the error streams created by the various pablo kernels
     * defined in `xml.pablo`.
     * 
     * Any bit in any of the streams in `Errors` is converted to an error message
     * via a `Code` and logged.
     */
    { // Error Stream Processing Scope
        StreamSet * const Errs = su::Collapse(P, Errors);
        StreamSet * const Indices = scan::ToIndices(P, Errs);
        StreamSet * const LineNumbers = scan::LineNumbers(P, Errs, LineBreakStream);
        StreamSet * const Spans = scan::FilterLineSpans(P, LineNumbers, LineSpans);
        StreamSet * const Codes = su::MultiplexWithMask(P, su::Select(P, Errors, su::Range(0, 8)), /*mask:*/ Errs);
        scan::Reader(P, pxDriver,  
            SCAN_CALLBACK(postproc_errorStreamsCallback),
            ByteStream, 
            { Indices, Spans },
            { LineNumbers, Codes });
    } // End Error Stream Processing Scope


#define POSTPROCESS_SCAN_KERNEL(SCAN_STREAM, CALLBACK) \
{ \
    StreamSet * const ScanStream = SCAN_STREAM; \
    StreamSet * const ScanPositions = scan::ToIndices(P, ScanStream); \
    StreamSet * const LineNumbers = scan::LineNumbers(P, ScanStream, LineBreakStream); \
    StreamSet * const Spans = scan::FilterLineSpans(P, LineNumbers, LineSpans); \
    scan::Reader(P, pxDriver, SCAN_CALLBACK(CALLBACK), ByteStream, { ScanPositions, Spans }, { LineNumbers }); \
}

    POSTPROCESS_SCAN_KERNEL(su::Select(P, CheckStreams, 0), postproc_validateNameStart);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, CheckStreams, 1), postproc_validateName);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, CCPCallouts, 4), postproc_validatePIName);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, CCPCallouts, 2), postproc_validateCDATA);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, RefCallouts, 0), postproc_validateGenRef);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, RefCallouts, 2), postproc_validateDecRef);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, RefCallouts, 4), postproc_validateHexRef);
    POSTPROCESS_SCAN_KERNEL(su::Select(P, CheckStreams, 2), postproc_validateAttRef);

    /*
        XML Declaration Parsing
     */
    {
        StreamSet * const Marker = su::Select(P, CCPCallouts, 8);
        StreamSet * const Indices = scan::ToIndices(P, Marker);
        scan::Reader(P, pxDriver,
            SCAN_CALLBACK(postproc_validateXmlDecl),
            ByteStream,
            { Indices },
            { Indices });
    }

    return reinterpret_cast<XMLProcessFunctionType>(P->compile());
}


int main(int argc, char ** argv) {
    codegen::ParseCommandLineOptions(argc, argv, {&xmlFlags, pablo_toolchain_flags(), codegen::codegen_flags()});

    struct stat sb;
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (errno == EACCES) {
            std::cerr << "xml: " << inputFile << ": Permission denied.\n";
        }
        else if (errno == ENOENT) {
            std::cerr << "xml: " << inputFile << ": No such file.\n";
        }
        else {
            std::cerr << "xml: " << inputFile << ": Failed.\n";
        }
        return errno;
    }
    if (stat(inputFile.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        std::cerr << "xml: " << inputFile << ": Is a directory.\n";
        close(fd);
        return -1;
    }

    CPUDriver pxDriver("xml-pablo");
    auto em = ErrorManager::Create();
    auto parser = PabloParser::Create(Lexer::Create(em), em);
    auto xmlSource = SourceFile::Relative("xml.pablo");
    if (xmlSource == nullptr) {
        std::cerr << "pablo-parser: error loading pablo source file: xml.pablo\n";
    }
    auto xmlProcessFuncPtr = xmlPipelineGen(pxDriver, parser, xmlSource);

    xmlProcessFuncPtr(fd);
    ShowError();
    close(fd);
}
