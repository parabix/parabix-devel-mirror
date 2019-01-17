#ifndef OPTIMIZATIONBRANCH_H
#define OPTIMIZATIONBRANCH_H

#include <kernels/kernel.h>

namespace llvm { class Value; }

namespace kernel {

struct OptimizationBranchCompiler;

class OptimizationBranch final : public Kernel {
    friend class OptimizationBranchBuilder;
public:

    static bool classof(const Kernel * const k) {
        switch (k->getTypeId()) {
            case TypeId::OptimizationBranch:
                return true;
            default:
                return false;
        }
    }

    const static std::string CONDITION_TAG;

    ~OptimizationBranch();

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

    void addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) final;

    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) final;

private:

    llvm::Value * getItemCountIncrement(const std::unique_ptr<KernelBuilder> & b, const Binding & binding,
                                        llvm::Value * const first, llvm::Value * const last,
                                        llvm::Value * const defaultValue = nullptr) const;

    void callKernel(const std::unique_ptr<KernelBuilder> & b,
                    const Kernel * const kernel, llvm::Value * const first, llvm::Value * const last,
                    llvm::PHINode * const terminatedPhi);

private:

    Relationship * const                                mCondition;
    Kernel * const                                      mNonZeroKernel;
    Kernel * const                                      mAllZeroKernel;
    mutable std::unique_ptr<OptimizationBranchCompiler> mCompiler;
};

}

#endif // OPTIMIZATIONBRANCH_H
