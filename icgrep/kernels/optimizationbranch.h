#ifndef OPTIMIZATIONBRANCH_H
#define OPTIMIZATIONBRANCH_H

#include <kernels/core/kernel.h>

namespace llvm { class Value; }

namespace kernel {

class OptimizationBranchCompiler;

class OptimizationBranch final : public Kernel {
    friend class OptimizationBranchBuilder;
    friend class OptimizationBranchCompiler;
public:

    static bool classof(const Kernel * const k) {
        switch (k->getTypeId()) {
            case TypeId::OptimizationBranch:
                return true;
            default:
                return false;
        }
    }

    const Kernel * getAllZeroKernel() const {
        return mAllZeroKernel;
    }

    const Kernel * getNonZeroKernel() const {
        return mNonZeroKernel;
    }

    Relationship * getCondition() const {
        return mCondition;
    }

protected:

    OptimizationBranch(const std::unique_ptr<KernelBuilder> & b,
                       std::string && signature,
                       not_null<Relationship *> condition,
                       Kernel * const nonZeroKernel,
                       Kernel * const allZeroKernel,
                       Bindings && stream_inputs,
                       Bindings && stream_outputs,
                       Bindings && scalar_inputs,
                       Bindings && scalar_outputs);

    void addInternalProperties(const std::unique_ptr<kernel::KernelBuilder> & b) final;

    void addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) final;

    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateInitializeThreadLocalMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateFinalizeThreadLocalMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) final;

private:

    Relationship * const                        mCondition;
    Kernel * const                              mNonZeroKernel;
    Kernel * const                              mAllZeroKernel;
    std::unique_ptr<OptimizationBranchCompiler> mCompiler;
};

}

#endif // OPTIMIZATIONBRANCH_H
