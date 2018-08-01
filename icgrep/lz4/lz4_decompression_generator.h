
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


typedef void (*MainFunctionType)(char * byte_data, size_t headerSize, size_t filesize, bool hasBlockChecksum);



class LZ4DecompressionGenerator: public LZ4BaseGenerator {

public:
    LZ4DecompressionGenerator();

    int decompress(std::string&& inputFileName, std::string&& outputFileName, bool overwriteOutput);

    MainFunctionType getMainFunc();

    void generateDecompressionPipeline(const std::string &outputFile);
protected:
    void generateMainFunc(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

};


#endif //ICGREP_LZ4GENERATOR_H
