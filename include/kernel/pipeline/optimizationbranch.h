#ifndef OPTIMIZATIONBRANCH_H
#define OPTIMIZATIONBRANCH_H

#include <kernel/core/kernel.h>

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

    OptimizationBranch(BuilderRef b,
                       std::string && signature,
                       not_null<Relationship *> condition,
                       Kernel * const nonZeroKernel,
                       Kernel * const allZeroKernel,
                       Bindings && stream_inputs,
                       Bindings && stream_outputs,
                       Bindings && scalar_inputs,
                       Bindings && scalar_outputs);

    void addInternalProperties(BuilderRef b) final;

    void addKernelDeclarations(BuilderRef b) final;

    void generateInitializeMethod(BuilderRef b) final;

    void generateInitializeThreadLocalMethod(BuilderRef b) final;

    void generateKernelMethod(BuilderRef b) final;

    void generateFinalizeThreadLocalMethod(BuilderRef b) final;

    void generateFinalizeMethod(BuilderRef b) final;

private:

    Relationship * const                        mCondition;
    Kernel * const                              mNonZeroKernel;
    Kernel * const                              mAllZeroKernel;
    std::unique_ptr<OptimizationBranchCompiler> mCompiler;
};

}

#endif // OPTIMIZATIONBRANCH_H
