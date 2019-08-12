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

#include "stream_parsing.hpp"

namespace testing {

namespace {

constexpr char ReverseHexDigitTable[] = {
    '0', '8', '4', 'c', '2', 'a', '6', 'e', 
    '1', '9', '5', 'd', '3', 'b', '7', 'f'
};

constexpr char HexDigitTable[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', 
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

} // anonymous namspace

InMemoryStream<> HexStream(StaticStream stream) {
    std::vector<uint8_t> hex{};
    auto node = ssn::parse<ssn::HexConverter>(stream);
    ssn::ast::walk(node, [&](uint8_t v) {
        hex.push_back(ReverseHexDigitTable[(int) v]);
    });
    auto ims = InMemoryStream<>(std::move(hex));
    ims.setFieldWidth(1);
    return ims;
}


InMemoryStream<> BinaryStream(StaticStream stream) {
    std::vector<uint8_t> hex{};
    int counter = 0;
    uint8_t builder = 0;
    auto node = ssn::parse<ssn::BinaryConverter>(stream);
    ssn::ast::walk(node, [&](uint8_t v) {
        builder |= v << counter;
        counter++;
        if (counter == 4) {
            hex.push_back(HexDigitTable[(int) builder]);
            builder = 0;
            counter = 0;
        }
    });
    if (counter != 0) {
        hex.push_back(HexDigitTable[(int) builder]);
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
