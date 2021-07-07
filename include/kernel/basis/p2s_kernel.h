/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

#include <re/alphabet/alphabet.h>
#include <kernel/core/kernel.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class P2SKernel final : public BlockOrientedKernel {
public:
    P2SKernel(BuilderRef b,
              StreamSet * basisBits,
              StreamSet * byteStream);
private:
    void generateDoBlockMethod(BuilderRef b) override;
};


class P2SMultipleStreamsKernel final : public BlockOrientedKernel {
public:
    P2SMultipleStreamsKernel(BuilderRef b,
                             const StreamSets & inputStreams,
                             StreamSet * const outputStream);
protected:
    void generateDoBlockMethod(BuilderRef b) override;
private:
};

class P2SKernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2SKernelWithCompressedOutput(BuilderRef b);
private:
    void generateDoBlockMethod(BuilderRef b) override;
};

class P2S16Kernel final : public BlockOrientedKernel {
public:
    P2S16Kernel(BuilderRef b, StreamSet * u16bits, StreamSet * u16stream, cc::ByteNumbering endianness = cc::ByteNumbering::LittleEndian);
    P2S16Kernel(BuilderRef b, StreamSets & inputSets, StreamSet * u16stream, cc::ByteNumbering endianness = cc::ByteNumbering::LittleEndian);
private:
    void generateDoBlockMethod(BuilderRef b) override;
    cc::ByteNumbering mByteNumbering;
};

class P2S16KernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2S16KernelWithCompressedOutput(BuilderRef b,
                                    StreamSet * basisBits, StreamSet * fieldCounts, StreamSet * i16Stream, cc::ByteNumbering endianness = cc::ByteNumbering::LittleEndian);
private:
    void generateDoBlockMethod(BuilderRef b) override;
    cc::ByteNumbering mByteNumbering;
};

class P2S16KernelWithCompressedOutputOld final : public BlockOrientedKernel {
public:
    P2S16KernelWithCompressedOutputOld(BuilderRef b,
                                       StreamSet * basisBits, StreamSet * delCounts, StreamSet * byteStream);
private:
    void generateDoBlockMethod(BuilderRef b) override;
};

class P2S21Kernel final : public BlockOrientedKernel {
public:
    P2S21Kernel(BuilderRef b, StreamSet * u21bits, StreamSet * u32stream, cc::ByteNumbering = cc::ByteNumbering::LittleEndian);
private:
    void generateDoBlockMethod(BuilderRef b) override;
    cc::ByteNumbering mByteNumbering;
};

}

#endif
