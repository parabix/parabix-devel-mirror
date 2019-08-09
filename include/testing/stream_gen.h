/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <vector>
#include <type_traits>
#include <testing/common.h>
#include <llvm/IR/Type.h>
#include <llvm/Support/raw_ostream.h>

namespace testing {

/**
 * `field_width<I>::value` is the bit width of `I` where `I` is some integer type.
 */
template<typename I>
struct field_width {
    static_assert(std::is_integral<I>::value, "'I' template parameter must be an integer type");
    static const uint32_t value = sizeof(I) * 8;
};

/**
 * Type alias for a readonly character buffer. Used as an input parameter for
 * various stream generation functions.
 */
using StaticStream = const char *;

/**
 * Type alias for a set of readonly character buffers. Used as an input
 * parameter for various stream generating functions to produce a stream set
 * containing multiple streams.
 */
using StaticStreamSet = std::initializer_list<const char *>;

/**
 * `InMemoryStream` holds a stream buffer which can be converted to a usable
 * `StreamSet` instance via a call to `MemorySourceKernel`. Such calls are
 * abstracted behind the `BitStreamSource` and `ByteStreamSource` functions.
 * 
 * Not intended to be constructed directly, use the `HexStream`, `BinaryStream`
 * or `TextStream` functions to construct an instance of this type.
 */
template<typename I = uint8_t>
class InMemoryStream {
    static_assert(std::is_integral<I>::value, "'I' template parameter must be an integer type");
public:
    InMemoryStream() = delete;
    InMemoryStream(const InMemoryStream &) = delete;
    InMemoryStream(InMemoryStream && other) noexcept
    : buffer(std::move(other.buffer)), _size(other._size) {}

    InMemoryStream(std::vector<I> && b)
    : buffer(b), _size(buffer.size()) {}

    const I * raw() const { return buffer.begin().base(); }
    size_t size() const noexcept { return _size; }

    size_t fieldWidth() const noexcept { return fw; }
    void setFieldWidth(uint32_t fw) noexcept { this->fw = fw; }

private:
    std::vector<I> buffer;
    size_t         _size;
    uint32_t       fw = field_width<I>::value;
};

/**
 * Converts a string of hexadecimal digits into a format which can be converted
 * to a usable bitstream `StreamSet`.
 * 
 * Any whitespace in the string will simply be skipped over.
 * 
 * The conversion process from digit string to bitstream involves interpreting
 * each hex digit as its binary representation (e.g., 'c' -> '1100') and storing
 * the data in such a way that the bit order is preserved. Bitstreams written in
 * hex should be read by reading from left to right, replacing each hex digit
 * with its binary representation.
 * This process can be seen in the example bellow, where the input string "cab1"
 * is converted to a bitstream then printed using `PrintRegister`. The printed
 * output, "... 00 00 8d 53", is read from right to left, where as the input
 * string is read from left to right.
 * 
 * Example:
 * 
 *      auto H = HexStream("cab1");
 * 
 *  Creates a bitstream: 11.. 1.1. 11.1 ...1 (read from left to right)
 *  Which, when printed using `PrintRegister` or `util::DebugDisplay`, gives:
 * 
 *      hex = ... 00 00 8d 53
 * 
 *  Note the reverse ording of bits as the printed output is read from right 
 *  to left.
 */
InMemoryStream<> HexStream(StaticStream stream);

/**
 * Converts a string of binary digits into a format which can be converted to a
 * usable bitstream `StreamSet`.
 * 
 * Dot characters ('.') may be used in place of '0' characters to make the 1 bits
 * stand out more. Along with this, any whitespace in the string will be skipped
 * over.
 * 
 * Example:

 *      auto B = BinaryStream("11.. 1.1. 1.11 ...1");
 * 
 *  Which, when printed using `PrintRegister` or `util::DebugDisplay`, gives:
 * 
 *      bin = ... 00 00 8d 53
 */
InMemoryStream<> BinaryStream(StaticStream stream);

/**
 * Packages up a string of characters without any processing into a format which
 * can be converted to a streamset of type `<i8>[1]`.
 * 
 * Example:
 * 
 *      auto Text = TextStream(
 *          "<doc>\n"
 *          "   <node attr='hello'/>\n"
 *          "</doc>\n"
 *      );
 */
InMemoryStream<> TextStream(StaticStream stream);

/**
 * Packages up a sequence of integers into a format which can be converted into 
 * a streamset. The type of `I` determines the field width of the eventual
 * streamset (e.g., int16_t for field with of 16, int32_t for 32, etc.).
 */
template<typename I>
InMemoryStream<I> IntStream(std::initializer_list<I> && values);

/**
 * Returns the pointer type of a `InMemoryStream`'s buffer. Used when binding
 * test case pipeline inputs.
 * 
 * Bitstreams (i.e., `<i1>[1]`) return a pointer of type `i8*`.
 */
template<typename I>
llvm::Type * BufferTypeOf(TestEngine & T, InMemoryStream<I> const & stream);

/**
 * Converts a `pointer` and `size`, supplied from a pipline's input scalars,
 * into a `StreamSet`.
 */
StreamSet * ToStreamSet(TestEngine & T, uint32_t fieldWidth, Scalar * pointer, Scalar * size);

/* ---------- */

template<typename I>
llvm::Type * BufferTypeOf(TestEngine & T, InMemoryStream<I> const & stream) {
    auto t = llvm::IntegerType::getIntNPtrTy(T.driver().getContext(), field_width<I>::value);
    return t;
}

template<typename I>
InMemoryStream<I> IntStream(std::initializer_list<I> && values) {
    static_assert(std::is_integral<I>::value, "'I' template parameter must be an integer type");
    return InMemoryStream<I>(std::vector<I>(values));
}

}
