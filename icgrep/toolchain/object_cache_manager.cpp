#include "object_cache_manager.h"
#include <kernels/kernel.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/raw_ostream.h>

std::unique_ptr<ParabixObjectCache> ObjectCacheManager::mCache;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCacheSystems
 ** ------------------------------------------------------------------------------------------------------------- */
void ObjectCacheManager::initializeCacheSystems() {
    if (LLVM_LIKELY(mCache.get() == nullptr)) {
        if (LLVM_LIKELY(codegen::EnableObjectCache)) {
            if (codegen::ObjectCacheDir) {
                mCache.reset(new ParabixObjectCache(codegen::ObjectCacheDir));
            } else {
                mCache.reset(new ParabixObjectCache());
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
bool ObjectCacheManager::checkForCachedKernel(const std::unique_ptr<kernel::KernelBuilder> & b, kernel::Kernel * const kernel) noexcept {
    return mCache.get() && mCache->loadCachedObjectFile(b, kernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief performIncrementalCacheCleanupStep
 ** ------------------------------------------------------------------------------------------------------------- */
void ObjectCacheManager::performIncrementalCacheCleanupStep() noexcept {
    if (mCache.get()) mCache->performIncrementalCacheCleanupStep();
}
