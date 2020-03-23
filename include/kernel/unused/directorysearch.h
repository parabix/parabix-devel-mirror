#ifndef DIRECTORYSEARCH_H
#define DIRECTORYSEARCH_H

#include <kernel/core/kernel.h>

namespace kernel { class KernelBuilder; }

namespace kernel {

class DirectorySearch final : public SegmentOrientedKernel {
public:
    DirectorySearch(BuilderRef iBuilder,
                    Scalar * const rootPath,
                    StreamSet * const directoryNameStream, StreamSet * const fileDirectoryStream, StreamSet * const fileNameStream,
                    const unsigned filesPerSegment = 1024, const bool recursive = true, const bool includeHidden = false);

    void linkExternalMethods(BuilderRef b) override;

    void generateInitializeMethod(BuilderRef b) override;

    void generateDoSegmentMethod(BuilderRef b) override;

    void generateFinalizeMethod(BuilderRef b) override;
private:

    void addToOutputStream(BuilderRef b, llvm::Value * const name, llvm::Value * const nameLength, llvm::StringRef field, llvm::Value * const consumed);

private:
    const bool mRecursive;
    const bool mIncludeHidden;
    llvm::Function * fOpen;
    llvm::Function * fSysCall;
};

}

#endif // DIRECTORYSEARCH_H
