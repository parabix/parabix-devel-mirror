/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAM_SELECT_H
#define STREAM_SELECT_H

#include <initializer_list>
#include <utility>
#include <vector>
#include <unordered_map>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/core/kernel.h>

namespace kernel {

namespace __selops {

enum class __op { __select, __merge, __intersect };

template<typename Key>
struct __selop {
    using __param_bindings = std::vector<std::pair<Key, std::vector<uint32_t>>>;

    __op                operation;
    __param_bindings    bindings;
};

} // namespace __selops

using SelectOperation = __selops::__selop<StreamSet *>;

using SelectOperationList = std::vector<SelectOperation>;

using SelectedInput = __selops::__selop<std::string>;

using SelectedInputList = std::vector<SelectedInput>;


/**
 * Utility kernel allowing for moving, merging, and intersecting streams for
 * multiple streams sets.
 *
 * When using multiple operations, the order in which they appear in the
 * constructor determines their indices in the ouput stream set.
 *
 * Common Use Case Example:
 *
 *      StreamSet * const Subset = P->CreateStreamSet(2, 1);
 *      P->CreateKernelCall<StreamSelect>(Subset, Select(S, {0, 2}));
 *
 *      Where:
 *          S = { A, B, C, D }
 *          Subset = { A, C }
 *
 * Multi-Operation Example:
 *
 *      StreamSet * const Out = P->CreateStreamSet(5, 1);
 *      P->CreateKernelCall<StreamSelect>(Out, SelectOperationList {
 *          Select({ {X, {0, 1}}, {Y, {0}} }),
 *          Merge(Z, {0, 1, 2}),
 *          Select({ {Y, {1}} })
 *      });
 *
 *      Where:
 *          X = { A, B, C }
 *          Y = { D, E }
 *          Z = { F, G, H }
 *          Out = { A, B, D, F | G | H, E }
 *
 */
class StreamSelect : public BlockOrientedKernel {
public:
    using BuilderRef = BuilderRef;

    StreamSelect(BuilderRef b, StreamSet * output, SelectOperation operation);
    StreamSelect(BuilderRef b, StreamSet * output, SelectOperationList operations);

protected:
    void generateDoBlockMethod(BuilderRef b) override;

private:
    SelectedInputList mOperations;
};


/**
 * A variant of the `StreamSelect` kernel: reworked to support select operations
 * on integer streams.
 *
 * Unlike its bitstream counterpart, `IStreamSelect` does not support `merge`
 * or `intersect` operations. As such it does not support multi-operations.
 * Instead, its sole purpose is to provide support for moving streams between
 * stream sets.
 *
 * The `streamutils::Select` functions will automatically pick the correct
 * kernel to use based on their parameters.
 *
 * Preconditions:
 *  - All input stream sets must have the same field width (e.g., i8, i16, etc.)
 *
 * Returns:
 *  - A single stream set with the same field width as the inputs who's  layout
 *    is determined by the given `SelectOperation`.
 */
class IStreamSelect : public MultiBlockKernel {
public:
    using BuilderRef = BuilderRef;

    IStreamSelect(BuilderRef b, StreamSet * output, SelectOperation operation);

protected:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;

private:
    SelectedInput mOperation;
    uint32_t mFieldWidth = 0;
};


/**
 * Registers a select operation with a single source stream set.
 *
 * Examples:
 * (1) Subset:          Select(Source, {0, 2})
 *      Source: { A, B, C, D }
 *      Result: { A, C }
 *
 *  (2) Permutation:    Select(Source, {1, 3, 0, 2})
 *      Source: { A, B, C, D }
 *      Result: { B, D, A, C }
 */
SelectOperation Select(StreamSet * from, std::vector<uint32_t> indices);

/**
 * Registers a select operation with multiple source stream sets.
 *
 * Streams will be placed in the output stream in the order that they appear
 * in the function's parameters.
 *
 * Examples:
 * (1) Combine:         Select({ {X, {0, 1}}, {Y, {0, 1}} })
 *      X     : { A, B }
 *      Y     : { C, D }
 *      Result: { A, B, C, D }
 *
 * (2) Mix:             Select({ {X, {1}}, {Y, {0}}, {X, {1}} })
 *      X     : { A, B }
 *      Y     : { C, D }
 *      Result: { B, C, A }
 */
SelectOperation Select(std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings);

/**
 * Registers a select operation with multiple source stream sets.
 *
 * All streams for each stream set will be combined into a single stream set
 * with their order being determined by the order they appear in the function's
 * parameter.
 *
 * Examples:            Select({X, Y})
 *      X:      { A, B }
 *      Y:      { C, D, E }
 *      Result: { A, B, C, D, E }
 */
SelectOperation Select(std::vector<StreamSet *> sets);

/**
 * Registers a merge operation with a single source stream set.
 *
 * All selected streams will be merged into a single stream.
 *
 * Example:             Merge(Source, {0, 2})
 *      Source: { A, B, C, D }
 *      Result: { A | C }
 */
SelectOperation Merge(StreamSet * from, std::vector<uint32_t> indices);

/**
 * Registers a merge operation with multiple source stream sets.
 *
 * All selected streams will be merged into a single stream.
 *
 * Example:             Merge({ {X, {0, 1}}, {Y, {1}} })
 *      X     : { A, B }
 *      Y     : { C, D }
 *      Result: { A | B | D }
 */
SelectOperation Merge(std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings);

/**
 * Registers a merge operation with multiple source stream sets.
 *
 * All streams from each stream set will be merged into a single stream.
 *
 * Examples:            Merge({X, Y})
 *      X:      { A, B }
 *      Y:      { C, D, E }
 *      Result: { A | B | C | D | E }
 */
SelectOperation Merge(std::vector<StreamSet *> sets);

/**
 * Registers an intersect operation with a single source stream set.
 *
 * All selected streams will be intersected into a single stream.
 *
 * Example:             Intersect(Source, {0, 2})
 *      Source: { A, B, C, D }
 *      Result: { A & C }
 */
SelectOperation Intersect(StreamSet * from, std::vector<uint32_t> indices);

/**
 * Registers an intersect operation with multiple source stream sets.
 *
 * All selected streams will be intersected into a single stream.
 *
 * Example:             Intersect({ {X, {0, 1}}, {Y, {1}} })
 *      X     : { A, B }
 *      Y     : { C, D }
 *      Result: { A & B & D }
 */
SelectOperation Intersect(std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings);

/**
 * Registers an intersect operation with multiple source stream sets.
 *
 * All streams from each stream set will be intersected into a single stream.
 *
 * Examples:            Intersect({X, Y})
 *      X:      { A, B }
 *      Y:      { C, D, E }
 *      Result: { A & B & C & D & E }
 */
SelectOperation Intersect(std::vector<StreamSet *> sets);



namespace streamutils {

/**
 * Generates a vector of consecutive integers starting at an inclusive lower
 * bound and stopping at an exclusive upper bound.
 *
 * May be used as the `indices` parameter for stream operation functions.
 *
 * @param lb The inclusive lower bound.
 * @param ub The exclusive upper bound; must be strictly greater than `lb`.
 */
std::vector<uint32_t> Range(uint32_t lb, uint32_t ub);

/*
   Stream Set View Operations: Loading selected streams.

*/
std::string genSignature(SelectOperation const & operation);

std::string genSignature(SelectOperationList const & operations);

std::pair<SelectedInputList, std::unordered_map<StreamSet *, std::string>>
mapOperationsToStreamNames(SelectOperationList const & operations);

using BuilderRef = const std::unique_ptr<KernelBuilder> &;

std::vector<llvm::Value *> loadInputSelectionsBlock(BuilderRef b, SelectedInputList ops, llvm::Value * blockOffset);

/*
    Stream Set Generators

    Convenience functions which wrap around a call to the `StreamSelect` or
    `IStreamSelect` kernel. The size of the output stream set is automatically 
    calculated.

    Examples in this section assume a `ProgramBuilder` pointer `P` exists and
    the namespace alias`namespace su = kernel::streamutils;` is defined.
 */

/**
 * Selects a single stream from a given stream set and returns it as its own
 * stream set.
 *
 * Use of the `StreamSelect` or `IStreamSelect` kernel is determined based on
 * the field width of `from`.
 *
 * Preconditions:
 *  - `index` must be a valid stream index (i.e., within bounds)
 *
 * Returns:
 *  - A stream set containing the single stream from `from` at `index
 *      <iN>[1] where N is the field width of `from`
 *
 * Example:
 *      StreamSet * out = su::Select(P, in, 1);
 *
 *          in:     { A, B, C, D }
 *          out:    { B }
 *
 */
StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, uint32_t index);

/**
 * Selects multiple streams from a given stream set, packages them up into a new
 * stream set and returns it.
 *
 * The order of streams in the resultant stream set is determined by the order
 * indicies appear in the `indices` vector.
 *
 * Use of the `StreamSelect` or `IStreamSelect` kernel is determined based on
 * the field width of `from`.
 *
 * Preconditions:
 *  - All values in `indices` must be valid in `from` (i.e., within bounds)
 *
 * Returns:
 *  - A stream set containing only the streams with indicies specified by `indicies`
 *      <iN>[M] where
 *          N is the field width of `from`
 *          M is the number of streams selected (`indices.size()`)
 *
 * Example:
 *      StreamSet * out = su::Select(P, in, {2, 0, 3});
 *
 *          in:     { A, B, C, D }
 *          out:    { C, A, D }
 */
StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, std::vector<uint32_t> indices);

/**
 * Selects multple streams from multiple different stream sets, packages them
 * into a new stream set and returns it.
 *
 * The order of streams in the resultant stream set is determined by the order
 * streams appear in the outer vector, as well as the order that indicies are
 * appear in the inner vectors (see example).
 *
 * Use of the `StreamSelect` or `IStreamSelect` kernel is determined based on
 * the field width of the input stream sets.
 *
 * Preconditions:
 *  - All index values must be valid within the stream set they are selecting
 *    from.
 *  - All input stream sets must have the same field width: you can't mix and
 *    match i1 and i64 streams for example
 *
 * Returns:
 *  - A stream set containing the selected streams (see example)
 *      <iN>[M] where
 *          N is the field width for the input streams
 *          M is the number of selected streams
 *
 * Example:
 *      StreamSet * out = su::Select(P, {
 *          {in1, {2, 0}},
 *          {in0, {1}},
 *          {in2, {0, 1}}
 *      });
 *
 *          in0:    { A, B, C }
 *          in1:    { D, E, F }
 *          in2:    { G, H, I }
 *          out:    { C, A, E, G, I }
 */
StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections);

/**
 * Combines all streams from each stream set in `sets` into a new stream set.
 *
 * The order of streams in the resultant stream set is determined by the order
 * stream sets appear in `sets`. Streams are not re-ordred within sets.
 *
 * Use of the `StreamSelect` or `IStreamSelect` kernel is determined based on
 * the field width of the input stream sets.
 *
 * Preconditions:
 *  - All stream sets must have the same field width
 *
 * Returns:
 *  - A stream set containing all streams from the input stream sets
 *      <iN>[M] where
 *          N is the field width for the input streams
 *          M is the sum of the number of streams in each input stream set
 *
 * Example:
 *      StreamSet * out = su::Select(P, { in1, in0 });
 *
 *          in0:    { A, B }
 *          in1:    { C }
 *          out:    { C, A, B }
 */
StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, std::vector<StreamSet *> sets);

/**
 * Merges the streams from the stream set `from`, indexed by values in
 * `indicies`, into a single stream which is returned as a new stream set.
 *
 * Preconditions:
 *  - All values in `indices` must be valid in `from` (i.e., within bounds)
 *  - `from` must have a field width of 1 (i.e., <i1>[N])
 *
 * Returns:
 *  - A stream set containing a single stream: <i1>[1]
 *
 * Example:
 *      SteramSet * out = su::Merge(P, in, {0, 2});
 *
 *          in:     { A, B, C, D }
 *          out:    { (A | C) }
 */
StreamSet * Merge(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, std::vector<uint32_t> indices);

/**
 * Merges specific streams from multiple different stream sets into a single
 * stream which is returned as a new stream set.
 *
 * Preconditions:
 * - All index values must be valid within the stream set they are selecting
 *    from.
 * - All input streams must have a field width of 1 (i.e., <i1>[N])
 *
 * Returns:
 *  - A stream set containing a single stream: <i1>[1]
 *
 * Example:
 *      StreamSet * out = su::Merge(P, {
 *          { in0, {0, 2} },
 *          { in1, {1}}
 *      });
 *
 *          in0:    { A, B, C }
 *          in1:    { D, E, F }
 *          out:    { (A | C | E) }
 */
StreamSet * Merge(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections);

/**
 * Intersects the streams from the stream set `from`, indexed by values in
 * `indicies`, into a single stream which is returned as a new stream set.
 *
 * Preconditions:
 *  - All values in `indices` must be valid in `from` (i.e., within bounds)
 *  - `from` must have a field width of 1 (i.e., <i1>[N])
 *
 * Returns:
 *  - A stream set containing a single stream: <i1>[1]
 *
 * Example:
 *      SteramSet * out = su::Intersect(P, in, {0, 2});
 *
 *          in:     { A, B, C, D }
 *          out:    { (A & C) }
 */
StreamSet * Intersect(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, std::vector<uint32_t> indices);

/**
 * Intersects specific streams from multiple different stream sets into a single
 * stream which is returned as a new stream set.
 *
 * Preconditions:
 * - All index values must be valid within the stream set they are selecting
 *    from.
 * - All input streams must have a field width of 1 (i.e., <i1>[N])
 *
 * Returns:
 *  - A stream set containing a single stream: <i1>[1]
 *
 * Example:
 *      StreamSet * out = su::Intersect(P, {
 *          { in0, {0, 2} },
 *          { in1, {1}}
 *      });
 *
 *          in0:    { A, B, C }
 *          in1:    { D, E, F }
 *          out:    { (A & C & E) }
 */
StreamSet * Intersect(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections);

} // namespace kernel::streamutils

} // namespace kernel

#endif // STREAM_SELECT_H
