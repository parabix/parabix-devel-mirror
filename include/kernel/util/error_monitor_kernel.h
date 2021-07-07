/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef ERROR_MONITOR_KERNEL_H
#define ERROR_MONITOR_KERNEL_H

#include <initializer_list>
#include <kernel/core/kernel.h>

namespace kernel {

class ErrorMonitorKernel final : public MultiBlockKernel {
public:
    using IOStreamBindings = std::initializer_list<std::pair<StreamSet *, StreamSet *>>;

    ErrorMonitorKernel(BuilderRef b, StreamSet * error, IOStreamBindings bindings);
private:
    std::string mName;
    std::vector<std::pair<std::string, std::string>> mStreamNames;
    std::size_t mNextNameId = 0;

    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;

    std::pair<std::string, std::string> generateNewStreamSetNames() {
        std::size_t id = mNextNameId++;
        return {"in_" + std::to_string(id), "out_" + std::to_string(id)};
    }

    template<typename FuncTy>
    void foreachMonitoredStreamSet(FuncTy fn) {
        for (auto const & binding : mStreamNames) {
            fn(binding.first, binding.second);
        }
    }
};

} // namespace kernel

#endif // ERROR_MONITOR_KERNEL_H
