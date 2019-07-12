/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

#include <re/cc/alphabet.h>
#include <kernel/core/kernel.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class P2SKernel final : public BlockOrientedKernel {
public:
    P2SKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
              StreamSet * basisBits,
              StreamSet * byteStream);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
};


class P2SMultipleStreamsKernel final : public BlockOrientedKernel {
public:
    P2SMultipleStreamsKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                             const StreamSets & inputStreams,
                             StreamSet * const outputStream);
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
private:
};

class P2SKernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2SKernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & b);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
};

class P2S16Kernel final : public BlockOrientedKernel {
public:
    P2S16Kernel(const std::unique_ptr<kernel::KernelBuilder> &b, StreamSet * u16bits, StreamSet * u16stream, cc::ByteNumbering endianness = cc::ByteNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
    cc::ByteNumbering mByteNumbering;
};

class P2S16KernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2S16KernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & b,
                                    StreamSet * basisBits, StreamSet * fieldCounts, StreamSet * i16Stream, cc::ByteNumbering endianness = cc::ByteNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
    cc::ByteNumbering mByteNumbering;
};

class P2S21Kernel final : public BlockOrientedKernel {
public:
    P2S21Kernel(const std::unique_ptr<kernel::KernelBuilder> &b, StreamSet * u21bits, StreamSet * u32stream, cc::ByteNumbering = cc::ByteNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
    cc::ByteNumbering mByteNumbering;
};

}

#endif
