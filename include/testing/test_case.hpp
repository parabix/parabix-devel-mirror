/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

/*
    The contents of this file are meant to be internal to the testing framework
    and not accessed directly from unit test applications.
 */

#pragma once

#include <type_traits>
#include <tuple>
#include <string>
#include <vector>
#include <llvm/IR/Type.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/relationship.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <testing/stream_gen.hpp>
#include <util/apply.hpp>
#include <util/typelist.hpp>

namespace testing {

namespace tc {

using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

using ProgramBuilderRef = const std::unique_ptr<kernel::ProgramBuilder> &;

/*
    Pipeline Return Type Deduction

    Recursively constructs the function pointer type which will be returned by
    a pipeline with specified input types.

    Example:
        A pipeline with the following inputs:

            1. HexStreamSet (<i1>[N])
            2. uint16_t Scalar
            3. i64 Stream (<i64>[1])

        Will have the following return type:

            void(*)(
                const uint8_t *,    // pointer to param (1)'s stream buffer
                size_t,             // length of param (1)'s streams
                uint16_t,           // param (2)'s scalar value
                const uint64_t *,   // pointer to param (3)'s stream buffer
                size_t,             // length of param (3)'s stream
                int32_t *           // pointer to test result value (output)
            )
 */

/// Pointer to stream's buffer
template<typename I>
struct buffer_pointer {
    using type = const I *;
};

/// Pointer to stream's buffer
template<>
struct buffer_pointer<streamgen::bit_t> {
    using type = const uint8_t *;
};

/// Pointer to stream's buffer
template<>
struct buffer_pointer<streamgen::text_t> {
    using type = const uint8_t *;
};

/// Pointer to stream's buffer
template<typename I>
using buffer_pointer_t = typename buffer_pointer<I>::type;

/// Function pointer type accumulator.
template<typename... Params>
struct function_pointer {
    using type = void(*)(Params...);
};

template<template<typename...> class Accum, typename... Ts>
struct parameter_list {};

template<template<typename...> class Accum>
struct parameter_list<Accum> {
    using type = Accum<>;
};

template<template<typename...> class Accum, typename T, typename... Rest>
struct parameter_list<Accum, T, Rest...> {
    using recursive_t = typename parameter_list<Accum, Rest...>::type;
    using type = meta::typelist_prepend_t<Accum, T, recursive_t>;
};

template<template<typename...> class Accum, typename I, typename Decoder, typename... Rest>
struct parameter_list<Accum, streamgen::basic_stream<I, Decoder>, Rest...> {
    using recursive_t = typename parameter_list<Accum, Rest...>::type;
    using type = meta::typelist_prepend_t<Accum, buffer_pointer_t<I>, meta::typelist_prepend_t<Accum, size_t, recursive_t>>;
};

/// Pipeline parameter list type with a generic accumulator.
template<template<typename...> class Accum, typename... Ts>
using parameter_list_t = typename parameter_list<Accum, Ts...>::type;

/// Pipeline parameter list, including return pointer, with generic accumulator.
template<template<typename...> class Accum, typename... Ts>
using parameter_list_with_result_t = parameter_list_t<Accum, Ts..., int32_t *>;

/// The function pointer type of a compiled pipeline with specified parameters.
template<typename... PipelineParameters>
using pipeline_return_t = typename parameter_list_with_result_t<function_pointer, PipelineParameters...>::type;

/// The `tuple` type used as arguments to call a the corresponding `pipeline_return_t`.
template<typename... PipelineParameters>
using pipeline_invocation_pack_t = parameter_list_with_result_t<std::tuple, PipelineParameters...>;



/*
    Pipeline Runtime Type Deduction

    Creates a `std::tuple` type to contain the `kernel::StreamSet` and
    `kernel::Scalar` pipeline input values.
 */

template<typename I>
struct to_runtime_type {
    using type = kernel::Scalar *;
};

template<typename I, typename Decoder>
struct to_runtime_type<streamgen::basic_stream<I, Decoder>> {
    using type = kernel::StreamSet *;
};

/// A `std::tuple` which holds a pipeline's input values.
template<typename... PipelineParameters>
using runtime_type_pack_t = std::tuple<typename to_runtime_type<PipelineParameters>::type...>;



/*
    Input Binding Construction

    Uses template specialization matching to construct a chain of calls at
    compile time to generate the input bindings for a pipeline with specified
    parameter types.
 */

template<typename... Ts>
struct binding_list_construct_iterator {};

/// terminus case
template<>
struct binding_list_construct_iterator<> {
    static void call(BuilderRef b, kernel::Bindings & bindings) {
        bindings.push_back({b->getInt32Ty()->getPointerTo(), "output"});
    }
};

/// scalar case
template<typename I, typename... Rest>
struct binding_list_construct_iterator<I, Rest...> {
    static_assert(std::is_integral<I>::value, "`I` must be an integer type");
    static void call(BuilderRef b, kernel::Bindings & bindings) {
        auto i = bindings.size();
        auto name = "scalar-" + std::to_string(i);
        auto type = b->getIntNTy(streamgen::field_width<I>::value);
        bindings.push_back({type, name});
        binding_list_construct_iterator<Rest...>::call(b, bindings);
    }
};

/// stream case
template<typename I, typename Decoder, typename... Rest>
struct binding_list_construct_iterator<streamgen::basic_stream<I, Decoder>, Rest...> {
    static void call(BuilderRef b, kernel::Bindings & bindings) {
        auto i = std::to_string(bindings.size());
        auto ptr_name = "stream-ptr-" + i;
        auto len_name = "stream-len-" + i;
        auto buf_fw = streamgen::field_width<I>::value;
        auto ptr_type = llvm::IntegerType::getIntNPtrTy(b->getContext(), buf_fw);
        bindings.push_back({ptr_type, ptr_name});
        bindings.push_back({b->getSizeTy(), len_name});
        binding_list_construct_iterator<Rest...>::call(b, bindings);
    }
};

/// Constructs pipeline input bindings based on a set of `PipelineParameters`.
template<typename... PipelineParameters>
inline kernel::Bindings construct_bindings(BuilderRef b) {
    kernel::Bindings bindings{};
    binding_list_construct_iterator<PipelineParameters...>::call(b, bindings);
    return bindings;
}



/*
    Pipeline Call Argument Construction

    Constructs and populates the tuple used to invoke the compiled pipeline.
 */

template<typename... Ts>
struct argument_list_construct_iterator {};

template<>
struct argument_list_construct_iterator<> {
    using return_t = std::tuple<>;

    static return_t call() {
        return std::tuple<>();
    }
};

template<typename I, typename... Rest>
struct argument_list_construct_iterator<I, Rest...> {
    using recursive_call_t = argument_list_construct_iterator<Rest...>;
    using return_t = parameter_list_t<std::tuple, I, Rest...>;

    template<typename... Rs>
    static return_t call(I & i, Rs &... rest) {
        return std::tuple_cat(std::tuple<I>(i), recursive_call_t::call(rest...));
    }
};

template<typename I, typename Decoder, typename... Rest>
struct argument_list_construct_iterator<streamgen::basic_stream<I, Decoder>, Rest...> {
    using recursive_call_t = argument_list_construct_iterator<Rest...>;
    using return_t = parameter_list_t<std::tuple, streamgen::basic_stream<I, Decoder>, Rest...>;

    using stream_t = streamgen::basic_stream<I, Decoder>;

    template<typename... Rs>
    static return_t call(stream_t & stream, Rs &... rest) {
        return std::tuple_cat(
            std::make_tuple(stream.getBuffer().begin().base(), stream.getSize()),
            recursive_call_t::call(rest...)
        );
    }
};

/**
 * Takes in an arbitrary sequence of static stream references (e.g., HexStream,
 * IntStream<I>, etc.) and scalar (represented as primitive integers) references
 * and produces a `std::tuple` contating all of the need values to invoke a
 * compiled pipeline with the same `PipelineParameters` template parameters.
 */
template<typename... PipelineParameters>
inline
pipeline_invocation_pack_t<PipelineParameters...>
construct_pipeline_arguments(int32_t * result_ptr, PipelineParameters &... params) {
    return argument_list_construct_iterator<PipelineParameters..., int32_t *>::call(params..., result_ptr);
}



/*
    Construct Runtime Values

    Constructs `kernel::Scalar *` and `kernel::StreamSet *` values for a
    pipeline's inputs.
 */

template<size_t Index, typename... Ts>
struct runtime_val_construct_iterator {};

template<size_t Index>
struct runtime_val_construct_iterator<Index> {
    template<typename Tuple>
    static void call(Tuple &, ProgramBuilderRef, size_t) {}
};

template<size_t Index, typename I, typename... Rest>
struct runtime_val_construct_iterator<Index, I, Rest...> {
    using recursive_t = runtime_val_construct_iterator<Index + 1, Rest...>;

    // copy over scalar values with no processing
    template<typename Tuple>
    static void call(Tuple & out, ProgramBuilderRef P, size_t binding_idx) {
        std::get<Index>(out) = P->getInputScalar(binding_idx);
        recursive_t::call(out, P, binding_idx + 1);
    }
};

template<size_t Index, typename I, typename Decoder, typename... Rest>
struct runtime_val_construct_iterator<Index, streamgen::basic_stream<I, Decoder>, Rest...> {
    using recursive_t = runtime_val_construct_iterator<Index + 1, Rest...>;
    using stream_t = streamgen::basic_stream<I, Decoder>;

    // construct a streamset from the buffer pointer and size
    template<typename Tuple>
    static void call(Tuple & out, ProgramBuilderRef P, size_t binding_idx) {
        auto ptr = P->getInputScalar(binding_idx);
        auto len = P->getInputScalar(binding_idx + 1);
        auto fw = stream_t::field_width_v;
        auto count = stream_t::num_elements_v;
        auto streamset = ToStreamSet(P, fw, count, ptr, len);
        std::get<Index>(out) = streamset;
        recursive_t::call(out, P, binding_idx + 2);
    }
};

/// Constructs a tuple of `StreamSet` and `Scalar` pointers from a pipeline's inputs.
template<typename Tuple, typename... PipelineParameters>
inline
void
construct_pipeline_input_values(Tuple & out, ProgramBuilderRef P) {
    runtime_val_construct_iterator<0, PipelineParameters...>::call(out, P, 0);
}


template<typename... PipelineParameters>
class test_engine {
    using Self = test_engine<PipelineParameters...>;
public:
    using pipeline_input_pack_t = runtime_type_pack_t<PipelineParameters...>;

    test_engine()
    : mDriver("unit-test-framework")
    , mPipelineBuilder(mDriver.makePipeline(construct_bindings<PipelineParameters...>(mDriver.getBuilder())))
    {
        construct_pipeline_input_values<pipeline_input_pack_t, PipelineParameters...>(mPipelineInputs, mPipelineBuilder);
    }

    test_engine(const Self &) = delete;
    test_engine(Self &&) = delete;

    /// Returns the pipline input `StreamSet` or `Scalar` at a given `Index`.
    template<size_t Index>
    typename std::tuple_element<Index, pipeline_input_pack_t>::type
    input() const {
        return std::get<Index>(mPipelineInputs);
    }

    /// Returns a reference to the engine's internal `CPUDriver`.
    CPUDriver & driver() noexcept { return mDriver; }

    /// Returns a reference to the engine's internal `ProgramBuilder`.
    ProgramBuilderRef pipeline() noexcept { return mPipelineBuilder; }

    /// Returns a reference the the engine driver's `KernelBuilder`.
    BuilderRef builder() { return mDriver.getBuilder(); }

    /// Implicit cast to `ProgramBuilder` reference.
    operator ProgramBuilderRef () { return pipeline(); }

    /// Access to `ProgramBuilder`'s methods via the `->` operator.
    ProgramBuilderRef operator -> () { return pipeline(); }

private:
    CPUDriver                               mDriver;
    std::unique_ptr<kernel::ProgramBuilder> mPipelineBuilder;
    pipeline_input_pack_t                   mPipelineInputs;
};



template<typename... PipelineParameters>
class test_case {
public:
    using invocation_pack_t = pipeline_invocation_pack_t<PipelineParameters...>;
    using pipeline_fn_t = pipeline_return_t<PipelineParameters...>;
    using test_engine_t = test_engine<PipelineParameters...>;
    using pipeline_body_t = void(*)(test_engine_t &, ProgramBuilderRef);

    explicit test_case(PipelineParameters &... argv)
    : result(new int32_t(0))
    , invocation_pack(construct_pipeline_arguments(this->result, argv...))
    {}

    ~test_case() {
        delete result;
    }

    void set_body(pipeline_body_t body) {
        this->body = body;
    }

    void invoke() {
        assert(body != nullptr);
        // construct engine and pipeline
        test_engine_t T{};
        // populate pipline
        body(T, T.pipeline());
        // compile and invoke
        auto fn = reinterpret_cast<pipeline_fn_t>(T->compile());
        meta::apply(fn, std::move(invocation_pack));
    }

    int32_t get_result() const noexcept {
        return *result;
    }

private:
    int32_t *         result;
    invocation_pack_t invocation_pack;
    pipeline_body_t   body = nullptr;
};

template<typename... PipelineParameters>
inline
test_case<PipelineParameters...>
make_test_case(PipelineParameters &... argv) {
    return test_case<PipelineParameters...>(argv...);
}

} // namespace testing::tc

} // namespace testing



// --- Static Type Tests --- //

namespace testing {

namespace tc {

static_assert(
    std::is_same<
        void(*)(const uint8_t *, size_t, const uint8_t *, size_t, int32_t *),
        pipeline_return_t<
            streamgen::basic_stream<streamgen::bit_t, streamgen::hex_decoder>,
            streamgen::basic_stream<streamgen::bit_t, streamgen::hex_decoder>
        >
    >::value,
    "not the same"
);

static_assert(
    std::is_same<
        void(*)(const uint8_t *, size_t, int32_t *),
        pipeline_return_t<
            streamgen::basic_stream<streamgen::text_t, streamgen::hex_decoder>
        >
    >::value,
    "not the same"
);

static_assert(
    std::is_same<
        void(*)(const uint32_t *, size_t, uint64_t, const uint8_t *, size_t, int32_t *),
        pipeline_return_t<
            streamgen::basic_stream<uint32_t, streamgen::copy_decoder<uint32_t>>,
            uint64_t,
            streamgen::basic_stream<uint8_t, streamgen::copy_decoder<uint8_t>>
        >
    >::value,
    "not the same"
);

} // namespace testing::testcase

} // namespace testing
