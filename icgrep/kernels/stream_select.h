/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef STREAM_SELECT_H
#define STREAM_SELECT_H

#include <initializer_list>
#include <utility>
#include <vector>
#include <kernels/core/kernel.h>

namespace kernel {

namespace selops {

enum class __op { __select, __merge, __intersect };

template<typename Key>
struct __selop {
    using __param_bindings = std::vector<std::pair<Key, std::vector<uint32_t>>>;

    __op                operation;
    __param_bindings    bindings;
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
__selop<StreamSet *> Select(StreamSet * from, std::initializer_list<uint32_t> indices);

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
__selop<StreamSet *> Select(std::initializer_list<std::pair<StreamSet *, std::initializer_list<uint32_t>>> bindings);

/**
 * Registers a merge operation with a single source stream set.
 * 
 * All selected streams will be merged into a single stream.
 * 
 * Example:             Merge(Source, {0, 2})
 *      Source: { A, B, C, D }
 *      Result: { A | C }    
 */
__selop<StreamSet *> Merge(StreamSet * from, std::initializer_list<uint32_t> indices);

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
__selop<StreamSet *> Merge(std::initializer_list<std::pair<StreamSet *, std::initializer_list<uint32_t>>> bindings);

/**
 * Registers an intersect operation with a single source stream set.
 * 
 * All selected streams will be intersected into a single stream.
 * 
 * Example:             Intersect(Source, {0, 2})
 *      Source: { A, B, C, D }
 *      Result: { A & C }    
 */
__selop<StreamSet *> Intersect(StreamSet * from, std::initializer_list<uint32_t> indices);

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
__selop<StreamSet *> Intersect(std::initializer_list<std::pair<StreamSet *, std::initializer_list<uint32_t>>> bindings);

} // namespace selops

using SelectOperation = selops::__selop<StreamSet *>;

using SelectOperationList = std::initializer_list<SelectOperation>;

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

} // namespace kernel

#endif // STREAM_SELECT_H
