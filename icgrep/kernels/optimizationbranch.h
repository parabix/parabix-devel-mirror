#ifndef OPTIMIZATIONBRANCH_H
#define OPTIMIZATIONBRANCH_H

#include <kernels/kernel.h>

namespace llvm { class Value; }

namespace kernel {

struct OptimizationBranchCompiler;

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

    const Relationship * getCondition() const {
        return mCondition;
    }

protected:

    OptimizationBranch(const std::unique_ptr<KernelBuilder> & b,
                       std::string && signature,
                       not_null<Relationship *> condition,
                       not_null<Kernel *> nonZeroKernel,
                       not_null<Kernel *> allZeroKernel,
                       Bindings && stream_inputs,
                       Bindings && stream_outputs,
                       Bindings && scalar_inputs,
                       Bindings && scalar_outputs);

    void addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) final;

    void addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) final;

    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) final;

private:

    Relationship * const                        mCondition;
    Kernel * const                              mNonZeroKernel;
    Kernel * const                              mAllZeroKernel;
    std::unique_ptr<OptimizationBranchCompiler> mCompiler;
};

}

#endif // OPTIMIZATIONBRANCH_H
