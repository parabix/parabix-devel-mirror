/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAM_SELECT_H
#define STREAM_SELECT_H

#include <initializer_list>
#include <utility>
#include <vector>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/core/kernel.h>

namespace kernel {

namespace selops {

enum class __op { __select, __merge, __intersect };

template<typename Key>
struct __selop {
    using __param_bindings = std::vector<std::pair<Key, std::vector<uint32_t>>>;

    __op                operation;
    __param_bindings    bindings;
};

} // namespace selops

using SelectOperation = selops::__selop<StreamSet *>;

using SelectOperationList = std::vector<SelectOperation>;

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
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;

    StreamSelect(BuilderRef b, StreamSet * output, SelectOperation operation);
    StreamSelect(BuilderRef b, StreamSet * output, SelectOperationList operations);

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

protected:
    void generateDoBlockMethod(BuilderRef b) override;

private:
    std::vector<selops::__selop<std::string>> mOperations;
};


namespace streamops {

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


/* 
 * Stream Set Generators
 * 
 * Convenience functions which wrap around a call to the StreamSelect kernel.
 * The size of the output stream set is automatically calculated.
 */

StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, uint32_t index);

StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, std::vector<uint32_t> indices);

StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections);

StreamSet * Merge(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, std::vector<uint32_t> indices);

StreamSet * Merge(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections);

StreamSet * Intersect(const std::unique_ptr<ProgramBuilder> & P, StreamSet * from, std::vector<uint32_t> indices);

StreamSet * Intersect(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections);

} // namespace streamops

} // namespace kernel

#endif // STREAM_SELECT_H
