/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <boost/make_unique.hpp>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/core/streamset.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pablo_source_kernel.h>
#include <pablo/printer_pablos.h>
#include <pablo/parser/error.h>
#include <pablo/parser/lexer.h>
#include <pablo/parser/rd_parser.h>
#include <pablo/parser/simple_lexer.h>
#include <pablo/parser/pablo_parser.h>
#include <toolchain/cpudriver.h>

using namespace pablo::parse;
using namespace kernel;
using namespace llvm;


extern "C" void record_counts(int wc) {
    std::cout << "word count: " << wc << "\n";
}


typedef void (*WordCountFunctionType)(uint32_t fd);

WordCountFunctionType wcPipelineGen(CPUDriver & pxDriver, std::shared_ptr<PabloParser> parser, std::shared_ptr<SourceFile> source) {
    auto & iBuilder = pxDriver.getBuilder();
    Type * const int32Ty = iBuilder->getInt32Ty();
    auto P = pxDriver.makePipeline({Binding{int32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);
    auto CountableStream = ByteStream;
    auto BasisBits = P->CreateStreamSet(8, 1);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    CountableStream = BasisBits;
    Kernel * const wck = P->CreateKernelCall<pablo::PabloSourceKernel>(
        parser,
        source,
        std::string("WordCountKernel"),
        kernel::Bindings{{"countable", CountableStream}},
        kernel::Bindings{},
        kernel::Bindings{},
        kernel::Bindings{{iBuilder->getInt64Ty(), "wc"}}
    );
    Scalar * const wordCount = wck->getOutputScalar("wc");
    P->CreateCall("record_counts", record_counts, {wordCount});
    return reinterpret_cast<WordCountFunctionType>(P->compile());
}


int main(int argc, char ** argv) {
    if (argc < 3) {
        std::cerr << "a command and input file is required\n";
        return 1;
    }

    std::string filename{argv[2]};
    auto errorDelegate = std::make_shared<ErrorManager>();
    auto source = std::make_shared<SourceFile>(filename);

    // test lexer
    if (std::strcmp(argv[1], "tokenize") == 0) {
        SimpleLexer lexer(errorDelegate);
        auto tokens = lexer.tokenize(source);
        if (!tokens) {
            errorDelegate->dumpErrors();
            return 1;
        }

        for (auto const & token : tokens.value()) {
            std::cout << *token << "\n";
        }
    }

    // test parser
    else if (std::strcmp(argv[1], "wc") == 0) {
        if (argc != 4) {
            std::cerr << "USAGE: pablo wc <source file> <input file>\n";
            return 1;
        }
        CPUDriver pxDriver("pablo");
        auto lexer = boost::make_unique<SimpleLexer>(errorDelegate);
        auto parser = std::make_shared<RecursiveParser>(std::move(lexer), errorDelegate);
        auto source = std::make_shared<SourceFile>(std::string(argv[2]));
        std::string fileName(argv[3]);
        const int fd = open(argv[3], O_RDONLY);
        if (LLVM_UNLIKELY(fd == -1)) {
            if (errno == EACCES) {
                std::cerr << fileName << ": Permission denied.\n";
            }
            else if (errno == ENOENT) {
                std::cerr << "wc: " << fileName << ": No such file.\n";
            }
            else {
                std::cerr << "wc: " << fileName << ": Failed.\n";
            }
            return 1;
        }
        auto wc = wcPipelineGen(pxDriver, parser, source);
        wc(fd);
        close(fd);
    }

    else {
        std::cerr << "unknown command: " << argv[1] << "\n";
        return 1;
    }

    return 0;
}
