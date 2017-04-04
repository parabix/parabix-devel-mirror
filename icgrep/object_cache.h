/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef OBJECT_CACHE_H
#define OBJECT_CACHE_H

#include <llvm/ADT/SmallString.h>
#include <llvm/ExecutionEngine/ObjectCache.h>
#include <llvm/Support/MemoryBuffer.h>
#include <string>
#include <map>

namespace llvm { class Module; }

// The ParabixObjectCache is a two-level cache compatible with the requirements
// of the LLVM ExecutionEngine as well as the Parabix Kernel builder infrastructure.
//
// The ParabixObjectCache allows the ParabixDriver to look up cached modules based on a
// module stub that contains only the necessary Module ID and signature (loadCachedObjectFile).
// If found, the module object file is immediately loaded into the cachedObjectMap,
// and later made available to the ExecutionEngine as needed.  Otherwise, false is
// return to signal that a cached File is not found.  The ParabixDriver can then
// apply the necessary kernel builder to build the full module IR before passing
// it to the ExecutionEngine.
//

class ParabixObjectCache : public llvm::ObjectCache {
    public:
        ParabixObjectCache(const std::string &dir);
        ParabixObjectCache();
        virtual ~ParabixObjectCache();

        void notifyObjectCompiled(const llvm::Module *M, llvm::MemoryBufferRef Obj) override;
        bool loadCachedObjectFile(const llvm::Module* M);
        std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module* M) override;
    
    private:
        std::map<std::string, std::unique_ptr<llvm::MemoryBuffer>> cachedObjectMap;
        using Path = llvm::SmallString<128>;
        Path CacheDir;

        bool getCacheFilename(const std::string & ModID, Path & CacheName);
};

#endif
