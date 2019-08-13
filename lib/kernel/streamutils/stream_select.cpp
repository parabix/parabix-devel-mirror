/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/streamutils/stream_select.h>

#include <numeric>
#include <unordered_map>
#include <kernel/core/kernel_builder.h>
#include <llvm/Support/ErrorHandling.h>

using namespace llvm;

namespace kernel {

namespace __selops {

static std::string to_string(__op op) {
    switch (op) {
    case __op::__select:    return "select";
    case __op::__merge:     return "merge";
    case __op::__intersect: return "intersect";
    default:
        llvm_unreachable("invalid enum value");
        return "";
    }
}

static std::string to_string(StreamSet * ss, uint32_t index) {
    return "x" + std::to_string(ss->getNumElements()) + "@" + std::to_string(index);
}

static std::string to_string(__selop<StreamSet *> const & selop) {
    std::string s = to_string(selop.operation);
    uint32_t index = 0;
    for (auto binding : selop.bindings) {
        s += ":";
        s += to_string(binding.first, index);
        for (auto idx : binding.second) {
            s += std::to_string(idx);
        }
        s += ":";
        index++;
    }
    return s;
}

__selop<StreamSet *> __selop_init(__op op, StreamSet * from, std::vector<uint32_t> indices) {
    StreamSet * ss = from;
    typename __selop<StreamSet *>::__param_bindings bindings{};
    std::vector<uint32_t> idxVec{};
    idxVec.reserve(indices.size());
    for (auto i : indices) {
        assert ("invalid index" && i < ss->getNumElements());
        idxVec.push_back(i);
    }
    bindings.push_back(std::make_pair(ss, idxVec));
    return __selop<StreamSet *>{op, std::move(bindings)};
}

__selop<StreamSet *> __selop_init(__op op, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings) {
    typename __selop<StreamSet *>::__param_bindings b{};
    b.reserve(bindings.size());
    for (auto pair : bindings) {
        StreamSet * ss = pair.first;
        std::vector<uint32_t> indices{};
        indices.reserve(pair.second.size());
        for (auto index : pair.second) {
            assert ("invalid index" && index < ss->getNumElements());
            indices.push_back(index);
        }
        b.push_back(std::make_pair(ss, indices));
    }
    return __selop<StreamSet *>{op, std::move(b)};
}

__selop<StreamSet *> __selop_init(__op op, std::vector<StreamSet *> sets) {
    typename __selop<StreamSet *>::__param_bindings b{};
    for (auto set : sets) {
        std::vector<uint32_t> indices(set->getNumElements());
        std::iota(indices.begin(), indices.end(), 0);
        b.push_back(std::make_pair(set, indices));
    }
    return __selop<StreamSet *>{op, std::move(b)};
}

} // namespace __selops

SelectOperation Select(StreamSet * from, std::vector<uint32_t> indices) {
    return __selops::__selop_init(__selops::__op::__select, from, std::move(indices));
}

SelectOperation Select(std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings) {
    return __selops::__selop_init(__selops::__op::__select, std::move(bindings));
}

SelectOperation Select(std::vector<StreamSet *> sets) {
    return __selops::__selop_init(__selops::__op::__select, std::move(sets));
}


SelectOperation Merge(StreamSet * from, std::vector<uint32_t> indices) {
    return __selops::__selop_init(__selops::__op::__merge, from, std::move(indices));
}

SelectOperation Merge(std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings) {
    return __selops::__selop_init(__selops::__op::__merge, std::move(bindings));
}

SelectOperation Merge(std::vector<StreamSet *> sets) {
    return __selops::__selop_init(__selops::__op::__merge, std::move(sets));
}


SelectOperation Intersect(StreamSet * from, std::vector<uint32_t> indices) {
    return __selops::__selop_init(__selops::__op::__intersect, from, std::move(indices));
}

SelectOperation Intersect(std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> bindings) {
    return __selops::__selop_init(__selops::__op::__intersect, std::move(bindings));
}

SelectOperation Intersect(std::vector<StreamSet *> sets) {
    return __selops::__selop_init(__selops::__op::__intersect, std::move(sets));
}

static inline std::string genSignature(__selops::__selop<StreamSet *> const & operation) {
    return "_" + __selops::to_string(operation);
}

static std::string genSignature(std::vector<__selops::__selop<StreamSet *>> const & operations) {
    std::string s = "";
    for (auto const & op : operations) {
        s += "_";
        s += __selops::to_string(op);
    }
    return s;
}

static uint32_t resultStreamCount(__selops::__selop<StreamSet *> const & selop) {
    if (selop.operation == __selops::__op::__select) {
        // select operations return the same number of streams as the number of specified indices
        uint32_t rt = 0;
        for (auto const & binding : selop.bindings) {
            rt += binding.second.size();
        }
        return rt;
    } else {
        // merge and intersect operations return a single stream
        return 1;
    }
}

static uint32_t resultStreamFieldWidth(__selops::__selop<StreamSet *> const & selop) {
    assert (selop.bindings.size() > 0);
    uint32_t fw = 0;
    for (auto const & binding : selop.bindings) {
        if (fw == 0) {
            fw = binding.first->getFieldWidth();
            continue;
        }

        uint32_t x = binding.first->getFieldWidth();
        if (x != fw) {
            llvm::report_fatal_error("StreamSelect: mismatched field widths: " + std::to_string(x) + " vs " + std::to_string(fw));
        }
    }
    return fw;
}

static uint32_t resultStreamCount(std::vector<__selops::__selop<StreamSet *>> const & ops) {
    uint32_t count = 0;
    for (auto const & op : ops) {
        count += resultStreamCount(op);
    }
    return count;
}

// returns { mappedOperations, kernelBindings }
static std::pair<std::vector<__selops::__selop<std::string>>, std::unordered_map<StreamSet *, std::string>>
mapOperationsToStreamNames(__selops::__selop<StreamSet *> const & operation) {
    __selops::__selop<std::string> rt{};
    rt.operation = operation.operation;
    uint32_t idx = 0;
    std::unordered_map<StreamSet *, std::string> namingMap{};
    for (auto const & pair : operation.bindings) {
        std::string name;
        auto it = namingMap.find(pair.first);
        if (it != namingMap.end()) {
            name = it->second;
        } else {
            name = "set" + std::to_string(idx);
            idx++;
            namingMap.insert({pair.first, name});
        }
        rt.bindings.push_back({name, pair.second});
    }
    return std::make_pair(std::vector<__selops::__selop<std::string>>{rt}, namingMap);
}

// returns { mappedOperations, kernelBindings }
static std::pair<std::vector<__selops::__selop<std::string>>, std::unordered_map<StreamSet *, std::string>>
mapOperationsToStreamNames(std::vector<__selops::__selop<StreamSet *>> const & operations) {
    std::vector<__selops::__selop<std::string>> mapped{};
    mapped.reserve(operations.size());
    uint32_t idx = 0;
    std::unordered_map<StreamSet *, std::string> namingMap{};
    for (auto const & selop : operations) {
        __selops::__selop<std::string> mappedOp{};
        mappedOp.operation = selop.operation;
        for (auto const & pair : selop.bindings) {
            std::string name;
            auto it = namingMap.find(pair.first);
            if (it != namingMap.end()) {
                name = it->second;
            } else {
                name = "set" + std::to_string(idx);
                idx++;
                namingMap.insert({pair.first, name});
            }
            mappedOp.bindings.push_back({name, pair.second});
        }
        mapped.push_back(mappedOp);
    }
    return std::make_pair(mapped, namingMap);
}

StreamSelect::StreamSelect(BuilderRef b, StreamSet * output, SelectOperation operation) 
: BlockOrientedKernel(b, "StreamSelect" + genSignature(operation), {}, {{"output", output}}, {}, {}, {})
{
    assert (resultStreamCount(operation) == output->getNumElements());
    for (auto const & kv : operation.bindings) {
        if (kv.first->getFieldWidth() != 1) {
            llvm::report_fatal_error("StreamSelect: operations with this kernel are only supported for bitstreams");
        }
    }
    std::unordered_map<StreamSet *, std::string> inputBindings;
    std::tie(mOperations, inputBindings) = mapOperationsToStreamNames(operation);
    for (auto const & kv : inputBindings) {
        mInputStreamSets.push_back({kv.second, kv.first});
    }
}

StreamSelect::StreamSelect(BuilderRef b, StreamSet * output, SelectOperationList operations)
: BlockOrientedKernel(b, "StreamSelect" + genSignature(operations), {}, {{"output", output}}, {}, {}, {})
{
    assert (resultStreamCount(operations) == output->getNumElements());
    std::unordered_map<StreamSet *, std::string> inputBindings;
    std::tie(mOperations, inputBindings) = mapOperationsToStreamNames(operations);
    for (auto const & kv : inputBindings) {
        mInputStreamSets.push_back({kv.second, kv.first});
    }
}

void StreamSelect::generateDoBlockMethod(BuilderRef b) {
    uint32_t outIdx = 0;
    for (auto const & selop : mOperations) {
        if (selop.operation == __selops::__op::__select) {
            for (auto const & binding : selop.bindings) {
                std::string const & iStreamSetName = binding.first;
                for (auto const & index : binding.second) {
                    Value * const block = b->loadInputStreamBlock(iStreamSetName, b->getInt32(index));
                    b->storeOutputStreamBlock("output", b->getInt32(outIdx), block);
                    outIdx++;
                }
            }
        } else if (selop.operation == __selops::__op::__merge) {
            Value * accumulator = nullptr;
            for (auto const & binding : selop.bindings) {
                std::string const & iStreamSetName = binding.first;
                for (auto const & index : binding.second) {
                    Value * const block = b->loadInputStreamBlock(iStreamSetName, b->getInt32(index));
                    if (accumulator == nullptr) {
                        accumulator = block;
                    } else {
                        accumulator = b->simd_or(accumulator, block);
                    }
                }
            }
            b->storeOutputStreamBlock("output", b->getInt32(outIdx), accumulator);
            outIdx++;
        } else if (selop.operation == __selops::__op::__intersect) {
            Value * accumulator = nullptr;
            for (auto const & binding : selop.bindings) {
                std::string const & iStreamSetName = binding.first;
                for (auto const & index : binding.second) {
                    Value * const block = b->loadInputStreamBlock(iStreamSetName, b->getInt32(index));
                    if (accumulator == nullptr) {
                        accumulator = block;
                    } else {
                        accumulator = b->simd_and(accumulator, block);
                    }
                }
            }
            b->storeOutputStreamBlock("output", b->getInt32(outIdx), accumulator);
            outIdx++;
        } else {
            llvm_unreachable("invalid enum kernel::__selops::__op value");
        }
    }
}



IStreamSelect::IStreamSelect(BuilderRef b, StreamSet * output, SelectOperation operation)
: MultiBlockKernel(b, "IStreamSelect" + genSignature(operation), 
    {}, 
    {{"output", output, BoundedRate(0, 1)}}, 
    {}, {}, {})
{
    assert(resultStreamCount(operation) == output->getNumElements());
    std::unordered_map<StreamSet *, std::string> inputBindings;
    std::vector<__selops::__selop<std::string>> ops;
    std::tie(ops, inputBindings) = mapOperationsToStreamNames(operation);
    assert(ops.size() == 1);
    mOperation = ops[0];
    if (mOperation.operation != __selops::__op::__select) {
        llvm::report_fatal_error("IStreamSelect only supports the Select operation");
    }
    for (auto const & kv : inputBindings) {
        assert(mFieldWidth == 0 ? true : mFieldWidth == kv.first->getFieldWidth());
        mFieldWidth = kv.first->getFieldWidth();
        assert(mFieldWidth == output->getFieldWidth());
        mInputStreamSets.push_back({kv.second, kv.first, BoundedRate(0, 1)});
    }
    setStride(1);
}

void IStreamSelect::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    BasicBlock * const block_Entry = b->GetInsertBlock();
    BasicBlock * const block_Loop = b->CreateBasicBlock("loop");
    BasicBlock * const block_Exit = b->CreateBasicBlock("exit");
    Value * const initialStride = b->getProducedItemCount("output");
    b->CreateCondBr(mIsFinal, block_Exit, block_Loop);

    b->SetInsertPoint(block_Loop);
    PHINode * const strideNo = b->CreatePHI(b->getSizeTy(), 2);
    strideNo->addIncoming(b->getSize(0), block_Entry);
    uint32_t outIdx = 0;
    Value * const absPos = b->CreateAdd(strideNo, initialStride);
    for (auto const & binding : mOperation.bindings) {
        auto const & name = binding.first;
        for (auto const & idx : binding.second) {
            Value * const val = b->CreateLoad(b->getRawInputPointer(name, b->getInt32(idx), absPos));
            b->CreateStore(val, b->getRawOutputPointer("output", b->getInt32(outIdx), absPos));
            outIdx++;
        }
        b->setProcessedItemCount(name, b->CreateAdd(b->getProcessedItemCount(name), b->getSize(1)));
    }
    b->setProducedItemCount("output", b->CreateAdd(b->getProducedItemCount("output"), b->getSize(1)));
    Value * const nextStrideNo = b->CreateAdd(strideNo, b->getSize(1));
    strideNo->addIncoming(nextStrideNo, block_Loop);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), block_Loop, block_Exit);

    b->SetInsertPoint(block_Exit);
}


namespace streamutils {

using ProgramBuilderRef = const std::unique_ptr<ProgramBuilder> &;

std::vector<uint32_t> Range(uint32_t lb, uint32_t ub) {
    assert (lb < ub);
    std::vector<uint32_t> range{};
    range.reserve(ub - lb);
    for (; lb < ub; ++lb) {
        range.push_back(lb);
    }
    return range;
}

static StreamSet * runOperation(ProgramBuilderRef P, SelectOperation op) {
    uint32_t n = resultStreamCount(op);
    uint32_t fw = resultStreamFieldWidth(op);
    StreamSet * const output = P->CreateStreamSet(n, fw);
    if (fw == 1) {
        P->CreateKernelCall<StreamSelect>(output, op);
    } else if (op.operation == __selops::__op::__select) {
        P->CreateKernelCall<IStreamSelect>(output, op);
    } else {
        llvm::report_fatal_error("only Select operations are supported for streams with field width > 1");
    }
    return output;
}

StreamSet * Select(ProgramBuilderRef P, StreamSet * from, uint32_t index) {
    return Select(P, from, std::vector<uint32_t>{index});
}

StreamSet * Select(ProgramBuilderRef P, StreamSet * from, std::vector<uint32_t> indices) {
    SelectOperation op = Select(from, std::move(indices));
    return runOperation(P, std::move(op));
}

StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections) {
    SelectOperation op = Select(selections);
    return runOperation(P, std::move(op));
}

StreamSet * Select(const std::unique_ptr<ProgramBuilder> & P, std::vector<StreamSet *> sets) {
    SelectOperation op = Select(sets);
    return runOperation(P, std::move(op));
}

StreamSet * Merge(ProgramBuilderRef P, StreamSet * from, std::vector<uint32_t> indices) {
    SelectOperation op = Merge(from, std::move(indices));
    return runOperation(P, std::move(op));
}

StreamSet * Merge(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections) {
    SelectOperation op = Merge(selections);
    return runOperation(P, std::move(op));
}

StreamSet * Intersect(ProgramBuilderRef P, StreamSet * from, std::vector<uint32_t> indices) {
    SelectOperation op = Intersect(from, std::move(indices));
    return runOperation(P, std::move(op));
}

StreamSet * Intersect(const std::unique_ptr<ProgramBuilder> & P, std::vector<std::pair<StreamSet *, std::vector<uint32_t>>> selections) {
    SelectOperation op = Intersect(selections);
    return runOperation(P, std::move(op));
}

} // namespace streamops

} // namespace kernel
