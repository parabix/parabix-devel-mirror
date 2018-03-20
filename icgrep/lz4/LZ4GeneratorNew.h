

#ifndef ICGREP_LZ4GENERATORNEW_H
#define ICGREP_LZ4GENERATORNEW_H

#include "LZ4Generator.h"

class LZ4GeneratorNew: public LZ4Generator {
public:
    LZ4GeneratorNew();
protected:
    int get4MbBufferBlocks();
    virtual int getInputBufferBlocks() override;
    virtual int getDecompressedBufferBlocks() override;
    virtual void generateExtractAndDepositMarkers(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
};


#endif //ICGREP_LZ4GENERATORNEW_H
