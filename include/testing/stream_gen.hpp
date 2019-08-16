/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

/*
    This file implements the static stream type.
    Anything in `snake_case` is meant to be internal to this file.
 */

#pragma once

#include <iostream>

#include <cinttypes>
#include <type_traits>
#include <tuple>
#include <vector>
#include <llvm/IR/Type.h>
#include <llvm/Support/ErrorHandling.h>
#include <kernel/io/source_kernel.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/util/hex_convert.h>
#include <testing/common.h>
#include <testing/stream_parsing.hpp>
#include <toolchain/toolchain.h>

namespace testing {

// This namespace is meant for internal use only.
namespace streamgen {

/**
 * A marker type used to differentiate between bitstreams (streams with field 
 * width == 1), and streams of integers (field width > 1).
 */
struct bit_t {};

/// The field width of an integer type in bits.
template<typename I> 
struct field_width {
    static_assert(std::is_integral<I>::value, "field_width<I>: `I` must be an integer type");
    static const uint32_t value = sizeof(I) * 8; 
};

/// The field width of an integer type in bits.
template<> 
struct field_width<bit_t> { 
    static const uint32_t value = 1; 
};

/**
 * A collection of types associated with streams derived from a single item
 * type `I`.
 * 
 * Template Arguments:
 *  - `I`: A stream's item type to generate types for. `bit_t` for bit streams,
 *         some other integer type for int streams (e.g., uint64_t for <i64>).
 * 
 * Preconditions (static):
 *  - `I` must either be `bit_t` or `std::is_integral<I>::value` must be `true`.
 */
template<typename I>
struct stream_traits {
private:
    struct is_bitstream { static const bool value = std::is_same<I, bit_t>::value; };
public:
    static_assert(std::is_same<I, bit_t>::value || std::is_integral<I>::value, 
        "stream_traits<I>: `I` must be an integer type or bit_t");

    /// The item type for a stream.
    using item_type = I;
    
    /// Meta condition resolving to `true` iff trait types are for a bitstream.
    static const bool is_bitstream_v = is_bitstream::value;

    /// Item item type for a stream's internal buffer.
    using buffer_item_type = typename std::conditional<is_bitstream_v, uint8_t, I>::type;

    /// The field width of the stream (e.g., 1 for bitstreams, 8 for <i8>, etc.).
    static const uint32_t field_width_v = field_width<I>::value;

    /// The field width of the stream's internal buffer's items.
    static const uint32_t buffer_item_field_width_v = field_width<buffer_item_type>::value;

    /// Type that a single stream (i.e., <iN>[1]) is constructable from.
    using literal_t = typename std::conditional<is_bitstream_v, const char *, std::vector<I>>::type;

    /// Type that a stream set is constructable from.
    using set_literal_t = std::vector<literal_t>;

    /// The internal buffer type of the stream.
    using buffer_t = std::vector<buffer_item_type>;
};

/**
 * A decoder which simply copies the input `literal_t` to create a buffer.
 * 
 * Only works for types where `literal_t` is the same as `buffer_t`. One such
 * example is integer streams.
 * 
 * Template Arguments:
 *  - `I`: A stream's item type (e.g., `bit_t`, `uint64_t`, etc.).
 * 
 * Preconditions (static):
 *  - `stream_traits<I>::literal_t` and `stream_traits<I>::buffer_t` must be
 *    the same type.
 */
template<typename I>
struct copy_decoder {
    using traits = stream_traits<I>;
    using item_type = typename traits::item_type;
    using result_t = std::tuple<typename traits::buffer_t, size_t, uint32_t>;

    static result_t decode(typename traits::literal_t const & str) {
        static_assert(std::is_same<typename traits::literal_t, typename traits::buffer_t>::value,
            "copy_decoder cannot be used when literal_t != buffer_t");
        return std::make_tuple(str, str.size(), 1);
    }
};

constexpr char rev_hex_table[] = {
    '0', '8', '4', 'c', '2', 'a', '6', 'e', 
    '1', '9', '5', 'd', '3', 'b', '7', 'f'
};

constexpr char hex_table[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', 
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

/**
 * A decoder for converting a string of hexadecimal digits into a buffer.
 * 
 * May only be used for bitstreams.
 */
struct hex_decoder {
    using traits = stream_traits<bit_t>;
    using item_type = typename traits::item_type;
    using result_t = std::tuple<typename traits::buffer_t, size_t, uint32_t>;

    static result_t decode(typename traits::literal_t const & str) {
        traits::buffer_t buffer{};
        auto node = ssn::parse<ssn::HexConverter>(str);
        ssn::ast::walk(node, [&](uint8_t v) {
            buffer.push_back(rev_hex_table[(int) v]);
        });
        return std::make_tuple(buffer, buffer.size(), 1);
    }
};

/**
 * A decoder for converting a string of binary digits into a buffer.
 * 
 * May only be used for bitstreams.
 */
struct bin_decoder {
    using traits = stream_traits<bit_t>;
    using item_type = typename traits::item_type;
    using result_t = std::tuple<typename traits::buffer_t, size_t, uint32_t>;

    static result_t decode(typename traits::literal_t const & str) {
        std::vector<uint8_t> buffer{};
        int counter = 0;
        uint8_t builder = 0;
        auto node = ssn::parse<ssn::BinaryConverter>(str);
        ssn::ast::walk(node, [&](uint8_t v) {
            builder |= v << counter;
            counter++;
            if (counter == 4) {
                buffer.push_back(hex_table[(int) builder]);
                builder = 0;
                counter = 0;
            }
        });
        if (counter != 0) {
            buffer.push_back(hex_table[(int) builder]);
        }
        return std::make_tuple(buffer, buffer.size(), 1);
    }
};

/**
 * A decoder for creating stream sets instead of single streams. Takes another
 * decoder type which is used to decode the individual streams.
 * 
 * Template Arguments:
 *  - `Decoder`: A decoder type for decoding individual streams.
 */
template<class Decoder>
struct stream_set_decoder {
    using item_type = typename Decoder::item_type;
    using traits = stream_traits<item_type>;
    using result_t = std::tuple<typename traits::buffer_t, size_t, uint32_t>;

    static result_t decode(typename traits::set_literal_t const & set) {
        size_t const streamCount = set.size();
        size_t const blockWidth = codegen::BlockSize;

        std::vector<typename traits::buffer_t> buffers(streamCount);
        ssize_t len = -1;
        for (size_t i = 0; i < streamCount; ++i) {
            auto result = Decoder::decode(set[i]);
            buffers[i] = std::move(std::get<0>(result));
            auto length = std::get<1>(result);
            // Assert that all expanded streams are the same size
            if (len == -1) {
                len = (ssize_t) length;
            } else {
                if ((ssize_t) length != len) {
                    llvm::report_fatal_error("all streams in a streamset must be the same length");
                }
            }
        }

        // Construct the buffer in such a way that `MemorySourceKernel` reads the 
        // correct data for each stream in the set.
        size_t const length = (size_t) len;
        typename traits::buffer_t buffer{};
        size_t itemsPerChunk = blockWidth;
        size_t numPasses = std::ceil((double) length / itemsPerChunk);

        // For each pass, pull a fixed number of items from each expanded stream
        // and append them to the buffer.
        //
        // For example, on the first pass, pull 64 items from the first stream,
        // 64 from the second, and so on, from all of the streams. Then, on the
        // next pass, pull the next 64 items from each stream.
        for (size_t passIdx = 0; passIdx < numPasses; passIdx++) {
            for (auto const & buf : buffers) {
                for (size_t i = 0; i < itemsPerChunk; ++i) {
                    typename traits::buffer_item_type val = 0;
                    auto idx = i + (passIdx * itemsPerChunk);
                    if (LLVM_LIKELY(idx < length)) {
                        val = buf[idx];
                    }
                    // std::cout << val << " ";
                    buffer.push_back(val);
                }
                // std::cout << "\n";
            }
            // std::cout << "\n";
        }

        return std::make_tuple(buffer, length, streamCount);
    }
};

/// Meta conditional resolving to `true` if `Decoder` is some `StreamSetDecoder`.
template<class Decoder>
struct is_stream_set_decoder { static const bool value = false; };

/// Meta conditional resolving to `true` if `Decoder` is some `StreamSetDecoder`.
template<class Inner>
struct is_stream_set_decoder<stream_set_decoder<Inner>> { static const bool value = true; };

/**
 * The actual stream type implementation.
 * 
 * Since buffer initialization may be dependent on command line flags, the
 * stream's internal buffer is lazily generated.
 * 
 * Template Arguments:
 *  - `I`: The stream's item type (e.g., `bit_t`, `uint64_t`, etc.).
 *  - `Decoder`: The decoder to use when generating the internal buffer from
 *      a literal value.
 * 
 * Preconditions (static):
 *  - `I` must either be `bit_t` or be an integer type.
 */
template<typename I, class Decoder>
class basic_stream {
    using traits = stream_traits<I>;
    static const bool is_streamset_v = is_stream_set_decoder<Decoder>::value;
public:
    using literal_t = typename std::conditional<is_streamset_v, 
                        typename traits::set_literal_t, 
                        typename traits::literal_t>::type;

public:
    explicit basic_stream(literal_t && literal)
    : mLiteral(literal) {}

    /// The field width of this stream.
    uint32_t getFieldWidth() const noexcept {
        return traits::field_width_v;
    }

    uint32_t getBufferFieldWidth() const noexcept {
        return traits::buffer_item_field_width_v;
    }

    /// The number of elements in this stream.
    uint32_t getNumElements() {
        initialize();
        return mNumElements;
    }

    /// The number of items in this stream.
    size_t getSize() {
        initialize();
        return mSize;
    }

    /// A reference to this stream's internal buffer.
    typename traits::buffer_t const & getBuffer() {
        initialize();
        return mBuffer;
    }

private:
    void initialize() {
        if (!mIsInitialized) {
            std::tie(mBuffer, mSize, mNumElements) = Decoder::decode(mLiteral);
            mIsInitialized = true;
        }
    }

    literal_t                 mLiteral;
    uint32_t                  mNumElements;
    size_t                    mSize;
    typename traits::buffer_t mBuffer;
    bool                      mIsInitialized = false;
};

} // namespace testing::streamgen

namespace __sg = streamgen;

/**
 * Static-Stream type constructable from a string of hexadecimal digits.
 * 
 * 'Static stream notation' syntax is used to construct the stream. See
 * `stream_parsing.hpp` for more information and parser implementation.
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
using HexStream = __sg::basic_stream<__sg::bit_t, __sg::hex_decoder>;

/**
 * Static-StreamSet type constructable from a vector or strings of hexadecimal 
 * digits.
 * 
 * 'Static stream notation' syntax is used to construct the stream. See
 * `stream_parsing.hpp` for more information and parser implementation.
 * 
 * Usage Example:
 * 
 *      auto Set = HexStreamSet({
 *          "1234 abcd",
 *          "dcba 4321"
 *      });
 * 
 * All streams in the set must be the same size when expanded.
 * 
 * Length Mismatch Example:
 * 
 *      auto InvalidSet = HexStreamSet({
 *          "(ab){16}",
 *          "(abcd){4}" // <-- error: length mismatch, 32 vs. 16
 *      });
 */
using HexStreamSet = __sg::basic_stream<__sg::bit_t, __sg::stream_set_decoder<__sg::hex_decoder>>;

/**
 * Static-Stream type constructable from a string of binary digits.
 * 
 * 'Static stream notation' syntax is used to construct the stream. See
 * `stream_parsing.hpp` for more information and parser implementation.
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
using BinaryStream = __sg::basic_stream<__sg::bit_t, __sg::bin_decoder>;

/**
 * Static-StreamSet type constructable from a vector of strings of binary 
 * digits.
 * 
 * 'Static stream notation' syntax is used to construct the stream. See
 * `stream_parsing.hpp` for more information and parser implementation. Dot 
 * characters ('.') may be used in place of '0' characters to make the 1 bits
 * stand out more.
 * 
 * Usage Example:
 * 
 *      auto Set = BinaryStreamSet({
 *          "(1.1.){4} (...1){4}",
 *          "(...1){4} (1.1.){4}"
 *      });
 * 
 * All streams in the set must be the same size when expanded.
 * 
 * Length Mismatch Example:
 * 
 *      auto InvalidSet = HexStreamSet({
 *          "(..1.){8}",
 *          "((11..){2} 1){4}" // <-- error: length mismatch, 32 vs. 36
 *      });
 */
using BinaryStreamSet = __sg::basic_stream<__sg::bit_t, __sg::stream_set_decoder<__sg::bin_decoder>>;

/**
 * Static-Stream type constructable from a raw ascii text string. No processing
 * is done to the input string.
 * 
 * Example:
 * 
 *      auto Text = TextStream(
 *          "<doc>\n"
 *          "   <node attr='hello'/>\n"
 *          "</doc>\n"
 *      );
 */
using TextStream = __sg::basic_stream<char, __sg::copy_decoder<char>>;

/**
 * Static-Stream type constructable from a sequence of integers. The field width
 * of the resultant stream is determined by the types of the integers used (`I`).
 * 
 * Template Arguments:
 *  - `I`: Integer type to construct from, determines the field width of the
 *         resultant stream.
 * 
 * Example:
 * 
 *      auto IStream = IntStream<uint64_t>({1, 2, 3, 4, 5, 6, 7, 8, 9});
 */
template<typename I>
using IntStream = __sg::basic_stream<I, __sg::copy_decoder<I>>;

/**
 * Static-StreamSet type constructable from a vector of sequence of integers. 
 * The field width of the resultant stream is determined by the types of the 
 * integers used (`I`).
 * 
 * * All streams must contain the same number of items.
 * 
 * Template Arguments:
 *  - `I`: Integer type to construct from, determines the field width of the
 *         resultant stream.
 * 
 * Example:
 * 
 *      auto ISet = IntStreamSet<uint64_t>({
 *          { 1,  2,  3,  4,  5,  6,  7,  8,  9},
 *          {10, 20, 30, 40, 50, 60, 70, 80, 90}
 *      });
 */
template<typename I>
using IntStreamSet = __sg::basic_stream<I, __sg::stream_set_decoder<__sg::copy_decoder<I>>>;

/// The type of a stream's internal buffer as a LLVM type.
template<typename Stream>
inline llvm::Type * BufferTypeOf(TestEngine & T, Stream const & stream) {
    return llvm::IntegerType::getIntNPtrTy(T.driver().getContext(), stream.getBufferFieldWidth());
}

/// Constructs a `kernel::StreamSet` from a static stream's properties.
inline StreamSet * ToStreamSet(
    TestEngine & T, 
    uint32_t fieldWidth, 
    uint32_t numElements, 
    Scalar * pointer, 
    Scalar * size) 
{
    namespace su = kernel::streamutils;
    // bitstreams are first loaded as <i8>[1] then converted to <i1>[1] via `HexToBinary`
    StreamSet * source = T->CreateStreamSet(numElements, fieldWidth == 1 ? 8 : fieldWidth);
    T->CreateKernelCall<kernel::MemorySourceKernel>(pointer, size, source);
    if (fieldWidth == 1) {
        if (numElements == 1) {
            StreamSet * bin = T->CreateStreamSet(1, 1);
            T->CreateKernelCall<kernel::HexToBinary>(source, bin);
            source = bin;
        } else {
            // TODO: add streamset support into HexToBinary
            std::vector<StreamSet *> binaryStreams(numElements);
            std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings{};
            for (size_t i = 0; i < numElements; ++i) {
                auto bin = T->CreateStreamSet(1, 1);
                binaryStreams[i] = bin;
                T->CreateKernelCall<kernel::HexToBinary>(su::Select(T, source, i), binaryStreams[i]);
                bindings.push_back({binaryStreams[i], {0}});
            }
            source = su::Select(T, bindings);
        }
    }
    return source;
}

} // namespace testing
