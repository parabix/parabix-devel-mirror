#ifndef PIPELINEDRIVER_H
#define PIPELINEDRIVER_H

#include <toolchain/object_cache.h>

namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }

class ParabixObjectCache;

class ObjectCacheManager {
public:

    static bool checkForCachedKernel(const std::unique_ptr<kernel::KernelBuilder> & b, kernel::Kernel * const kernel) noexcept;

    static void performIncrementalCacheCleanupStep() noexcept;

    static void initializeCacheSystems();

    static ParabixObjectCache * getObjectCache() {
        return mCache.get();
    }

private:

    static std::unique_ptr<ParabixObjectCache> mCache;
};

#endif // PIPELINEDRIVER_H
