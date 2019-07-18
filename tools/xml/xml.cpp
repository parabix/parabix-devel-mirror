/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "post_process.h"
#include "test_suite_error.h"

#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/util/source_kernel.h>
#include <kernel/util/deletion.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/util/scanning.h>
#include <kernel/util/stream_select.h>
#include <kernel/util/streams_merge.h>
#include <kernel/util/streamset_collapse.h>
#include <kernel/util/linebreak_kernel.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Path.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/pablo_parser.h>
#include <pablo/parse/simple_lexer.h>
#include <pablo/parse/rd_parser.h>
#include <toolchain/cpudriver.h>
#include <toolchain/toolchain.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sys/stat.h>

namespace fs = boost::filesystem;
namespace so = kernel::streamops;

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
    StreamSet * const CCPCallouts = P->CreateStreamSet(8, 1);
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
    StreamSet * const TagCallouts = P->CreateStreamSet(9, 1);
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

    StreamSet * const PostProcStartIndices = P->CreateStreamSet(1, 64);
    P->CreateKernelCall<ScanIndexGenerator>(so::Select(P, PostProcessTagMatchingStreams, 0), PostProcStartIndices);
    StreamSet * const PostProcEndIndices = P->CreateStreamSet(1, 64);
    P->CreateKernelCall<ScanIndexGenerator>(so::Select(P, PostProcessTagMatchingStreams, 1), PostProcEndIndices);

    // Create Tag Matching Stream Codes
    StreamSet * const CodeStreams = so::Select(P, PostProcessTagMatchingStreams, so::Range(2, 5));
    StreamSet * const CollapsedCodeStreams = P->CreateStreamSet(1, 1);
    P->CreateKernelCall<CollapseStreamSet>(CodeStreams, CollapsedCodeStreams);
    StreamSet * const ParallelCodes = P->CreateStreamSet(3, 1);
    FilterByMask(P, CollapsedCodeStreams, CodeStreams, ParallelCodes);
    StreamSet * const Codes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ParallelCodes, Codes);

    Kernel * const tagMatcher = P->CreateKernelCall<ScanReader>(
        ByteStream, 
        so::Select(P, {{PostProcStartIndices, {0}}, {PostProcEndIndices, {0}}}), 
        "postproc_tagMatcher",
        AdditionalStreams{ PostProcStartIndices, Codes }
    );
    pxDriver.LinkFunction(tagMatcher, "postproc_tagMatcher", postproc_tagMatcher);


#define POSTPROCESS_SCAN_KERNEL(SCAN_STREAM, CALLBACK) \
{ \
    StreamSet * const ScanStream = SCAN_STREAM; \
    StreamSet * const ScanPositions = P->CreateStreamSet(1, 64); \
    P->CreateKernelCall<ScanIndexGenerator>(ScanStream, ScanPositions); \
    StreamSet * const LineNumbers = P->CreateStreamSet(1, 64); \
    P->CreateKernelCall<LineNumberGenerator>(ScanStream, LineBreakStream, LineNumbers); \
    Kernel * const k = P->CreateKernelCall<LineBasedReader>(ByteStream, ScanPositions, LineNumbers, LineSpans, #CALLBACK); \
    pxDriver.LinkFunction(k, #CALLBACK, CALLBACK); \
}

    StreamSet * const LineBreakStream = P->CreateStreamSet();
    P->CreateKernelCall<UnixLinesKernelBuilder>(ByteStream, LineBreakStream);

    StreamSet * const LineSpans = P->CreateStreamSet(2, 64);
    P->CreateKernelCall<LineSpanGenerator>(LineBreakStream, LineSpans);

    StreamSet * const CollapsedErrors = P->CreateStreamSet();
    P->CreateKernelCall<CollapseStreamSet>(Errors, CollapsedErrors);
    StreamSet * const CollapsedErrorsIndices = P->CreateStreamSet(1, 64);
    P->CreateKernelCall<ScanIndexGenerator>(CollapsedErrors, CollapsedErrorsIndices);
    StreamSet * const CollapsedErrorsLineNumbers = P->CreateStreamSet(1, 64);
    P->CreateKernelCall<LineNumberGenerator>(CollapsedErrors, LineBreakStream, CollapsedErrorsLineNumbers);
    // a little hack to allow the P2S kernel to be used: only select the first 8 error streams,
    // code 0 will be used for the last error stream
    StreamSet * const ParallelErrorCodes = P->CreateStreamSet(8, 1);
    FilterByMask(P, CollapsedErrors, so::Select(P, Errors, so::Range(0, 8)), ParallelErrorCodes);
    StreamSet * const ErrorCodes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ParallelErrorCodes, ErrorCodes);
    Kernel * const errorsReader = P->CreateKernelCall<LineBasedReader>(
        ByteStream, 
        CollapsedErrorsIndices, 
        CollapsedErrorsLineNumbers, 
        LineSpans, 
        "postproc_errorStreamsCallback", 
        AdditionalStreams{ErrorCodes}
    );
    pxDriver.LinkFunction(errorsReader, "postproc_errorStreamsCallback", postproc_errorStreamsCallback);

    POSTPROCESS_SCAN_KERNEL(so::Select(P, CheckStreams, 0), postproc_validateNameStart);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, CheckStreams, 1), postproc_validateName);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, CCPCallouts, 4), postproc_validatePIName);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, CCPCallouts, 2), postproc_validateCDATA);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, RefCallouts, 0), postproc_validateGenRef);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, RefCallouts, 2), postproc_validateDecRef);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, RefCallouts, 4), postproc_validateHexRef);
    POSTPROCESS_SCAN_KERNEL(so::Select(P, CheckStreams, 2), postproc_validateAttRef);

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
    auto parser = RecursiveParser::Create(SimpleLexer::Create(em), em);
    auto xmlSource = SourceFile::Relative("xml.pablo");
    if (xmlSource == nullptr) {
        std::cerr << "pablo-parser: error loading pablo source file: xml.pablo\n";
    }
    auto xmlProcessFuncPtr = xmlPipelineGen(pxDriver, parser, xmlSource);

    xmlProcessFuncPtr(fd);
    ShowError();
    close(fd);
}
