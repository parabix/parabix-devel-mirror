/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef OBJECT_CACHE_H
#define OBJECT_CACHE_H

#include <llvm/ADT/SmallString.h>
#include <llvm/ExecutionEngine/ObjectCache.h>
#include <llvm/ADT/StringRef.h>
#include <boost/container/flat_map.hpp>
#include <string>

namespace llvm { class Module; }
namespace llvm { class MemoryBuffer; }
namespace llvm { class MemoryBufferRef; }
namespace llvm { class LLVMContext; }
namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }

// The ParabixObjectCache is a two-level cache compatible with the requirements
// of the LLVM ExecutionEngine as well as the Parabix Kernel builder infrastructure.
//
// The ParabixObjectCache allows the CPUEngineInstance to look up cached modules based on a
// module stub that contains only the necessary Module ID and signature (loadCachedObjectFile).
// If found, the module object file is immediately loaded into the cachedObjectMap,
// and later made available to the ExecutionEngine as needed.  Otherwise, false is
// return to signal that a cached File is not found.  The CPUEngineInstance can then
// apply the necessary kernel builder to build the full module IR before passing
// it to the ExecutionEngine.
//

class ParabixObjectCache final : public llvm::ObjectCache {
    template <typename K, typename V>
    using Map = boost::container::flat_map<K, V>;
    using ModuleCache = Map<std::string, std::pair<llvm::Module *, std::unique_ptr<llvm::MemoryBuffer>>>;
    using Instance = std::unique_ptr<ParabixObjectCache>;
public:
    using Path = llvm::SmallString<128>;

    static bool checkForCachedKernel(const std::unique_ptr<kernel::KernelBuilder> & b, kernel::Kernel * const kernel) noexcept;

    static void initializeCacheSystems() noexcept;

    static ParabixObjectCache * getInstance() {
        return mInstance.get();
    }

    bool loadCachedObjectFile(const std::unique_ptr<kernel::KernelBuilder> & idb, kernel::Kernel * const kernel);

    void notifyObjectCompiled(const llvm::Module * M, llvm::MemoryBufferRef Obj) override;

    std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module * M) override;

protected:

    ParabixObjectCache();
    void loadCacheSettings() noexcept;
    void saveCacheSettings() noexcept;

private:
    void initiateCacheCleanUp() noexcept;
    bool requiresCacheCleanUp() noexcept;
private:
    static Instance     mInstance;
    ModuleCache         mCachedObject;
    Path                mCachePath;
};

#endif
