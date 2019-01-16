/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef NVPTXDRIVER_H
#define NVPTXDRIVER_H

#include <toolchain/driver.h>

class NVPTXDriver final : public BaseDriver {
    friend class CBuilder;
public:
    NVPTXDriver(std::string && moduleName);

    ~NVPTXDriver();

    void addKernel(Kernel * const kernel) override { }

    void generateUncachedKernels() { }

    void * finalizeObject(kernel::PipelineKernel * pipeline) override;

    bool hasExternalFunction(const llvm::StringRef /* functionName */) const override { return false; }

protected:

    NVPTXDriver(std::string && moduleName);

private:

    llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const override;

};

#endif
