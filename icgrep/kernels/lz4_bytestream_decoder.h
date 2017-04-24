/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef LZ4_BYTESTREAM_DECODER_H
#define LZ4_BYTESTREAM_DECODER_H

#include "kernel.h"

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class LZ4ByteStreamDecoderKernel : public BlockOrientedKernel {
public:
    LZ4ByteStreamDecoderKernel(IDISA::IDISA_Builder * iBuilder, size_t bufferSize);
protected:
    void generateDoBlockMethod() override;
private:
    size_t mBufferSize;
};

}

#endif  // LZ4_BYTESTREAM_DECODER_H
