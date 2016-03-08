#ifndef OBJECT_CACHE_H
#define OBJECT_CACHE_H

#include <string>

#include <llvm/IR/Module.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/ExecutionEngine/ObjectCache.h>

class ICGrepObjectCache : public llvm::ObjectCache {
    public:
        ICGrepObjectCache(const std::string& CacheDir);
        virtual ~ICGrepObjectCache();

        void notifyObjectCompiled(const llvm::Module *M, llvm::MemoryBufferRef Obj) override;
        std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module* M) override;

    private:
        std::string CacheDir;

        bool getCacheFilename(const std::string &ModID, std::string &CacheName);
};

#endif
