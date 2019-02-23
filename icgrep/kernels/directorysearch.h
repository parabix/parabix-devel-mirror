#ifndef DIRECTORYSEARCH_H
#define DIRECTORYSEARCH_H

#include <kernels/core/kernel.h>

namespace kernel { class KernelBuilder; }

namespace kernel {

class DirectorySearch final : public SegmentOrientedKernel {
public:
    DirectorySearch(const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
                    Scalar * const rootPath,
                    StreamSet * const directoryNameStream, StreamSet * const fileDirectoryStream, StreamSet * const fileNameStream,
                    const unsigned filesPerSegment = 1024, const bool recursive = true, const bool includeHidden = false);

    bool isCachable() const override { return true; }

    bool hasSignature() const override { return false; }

    void linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void generateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void generateDoSegmentMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void generateFinalizeMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
private:

    void addToOutputStream(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const name, llvm::Value * const nameLength, llvm::StringRef field, llvm::Value * const consumed);

private:
    const bool mRecursive;
    const bool mIncludeHidden;
    llvm::Function * fOpen;
    llvm::Function * fSysCall;
};

}

#endif // DIRECTORYSEARCH_H
