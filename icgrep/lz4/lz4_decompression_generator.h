
#ifndef ICGREP_LZ4GENERATOR_H
#define ICGREP_LZ4GENERATOR_H

#include "lz4_base_generator.h"

#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>

#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>

#include <toolchain/toolchain.h>

#include <toolchain/cpudriver.h>
#include <string>

namespace re { class CC; }


typedef void (*MainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum, const char * outputFileName);



class LZ4DecompressionGenerator final : public LZ4BaseGenerator {

public:
    LZ4DecompressionGenerator();

    int decompress(std::string && inputFileName, std::string&& outputFileName, bool overwriteOutput);

private:

    std::unique_ptr<kernel::ProgramBuilder> makeInternalPipeline();

    MainFunctionType generateDecompressionPipeline();

};


#endif //ICGREP_LZ4GENERATOR_H
