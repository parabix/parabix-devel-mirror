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

    std::unique_ptr<KernelCompiler> instantiateKernelCompiler(BuilderRef b) const final;

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

    void addKernelDeclarations(BuilderRef b) override;

    void addInternalProperties(BuilderRef b) override;

    void generateInitializeMethod(BuilderRef b) override;

    void generateInitializeThreadLocalMethod(BuilderRef b) override;

    void generateKernelMethod(BuilderRef b) override;

    void generateFinalizeThreadLocalMethod(BuilderRef b) override;

    void generateFinalizeMethod(BuilderRef b) override;

private:

    Relationship * const                        mCondition;
    Kernel * const                              mNonZeroKernel;
    Kernel * const                              mAllZeroKernel;

};

}

#endif // OPTIMIZATIONBRANCH_H
