/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/stream_gen.h>

#include <cstring>
#include <vector>
#include <kernel/io/source_kernel.h>
#include <kernel/util/hex_convert.h>
#include <llvm/Support/ErrorHandling.h>

namespace testing {

namespace {

// Mapping of hex digits as ascii chars to ascii chars representing the input
// hex digit with its bits reversed.
constexpr uint8_t ReverseHexDigitTable[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     '0',  '8',  '4',  'c',  '2',  'a',  '6',  'e',    '1',  '9', 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    0xff,  '5',  'd',  '3',  'b',  '7',  'f', 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff,  '5',  'd',  '3',  'b',  '7',  'f', 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

constexpr char HexDigitTable[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', 
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

} // anonymous namspace

InMemoryStream<> HexStream(StaticStream stream) {
    std::vector<uint8_t> hex{};
    size_t idx = 0;
    while (stream[idx] != '\0') {
        char c = stream[idx];
        idx++;
        if (std::isspace(c)) {
            continue;
        } else if (!std::isxdigit(c)) {
            llvm::report_fatal_error(std::string("'").append(1, c).append("' is not a valid hex digit"));
        }

        auto digit = ReverseHexDigitTable[(int) c];
        hex.push_back(digit);
    }
    auto ims = InMemoryStream<>(std::move(hex));
    ims.setFieldWidth(1);
    return ims;
}


InMemoryStream<> BinaryStream(StaticStream stream) {
    std::vector<uint8_t> hex{};
    size_t idx = 0;

    auto getBit = [&]() -> uint8_t {
        while (true) {
            if (stream[idx] == '\0') {
                return 0;
            }
            char c = stream[idx];
            idx++;
            if (std::isspace(c)) {
                continue;
            } else if (c == '1') {
                return 1;
            } else if (c == '.' || c == '0') {
                return 0;
            } else {
                llvm::report_fatal_error(std::string("'").append(1, c).append("' is not a valid binary digit"));
            }
        }
    };

    while (stream[idx] != '\0') {
        uint8_t val = 0;
        for (size_t i = 0; i < 4; ++i) {
            val |= getBit() << i;
        }
        hex.push_back(HexDigitTable[val]);
    }

    auto ims = InMemoryStream<>(std::move(hex));
    ims.setFieldWidth(1);
    return ims;
}


InMemoryStream<> TextStream(StaticStream stream) {
    size_t len = std::strlen(stream);
    auto begin = (const uint8_t *) stream;
    auto end = (const uint8_t *) stream + len;
    return InMemoryStream<>(std::vector<uint8_t>(begin, end));
}

StreamSet * ToStreamSet(TestEngine & T, uint32_t fieldWidth, Scalar * pointer, Scalar * size) {
    // bitstreams are first loaded as <i8>[1] then converted to <i1>[1] via `HexToBinary`
    StreamSet * source = T->CreateStreamSet(1, fieldWidth == 1 ? 8 : fieldWidth);
    T->CreateKernelCall<kernel::MemorySourceKernel>(pointer, size, source);
    if (fieldWidth == 1) {
        StreamSet * bin = T->CreateStreamSet(1, 1);
        T->CreateKernelCall<kernel::HexToBinary>(source, bin);
        source = bin;
    }
    
    return source;
}

} // namespace testing
