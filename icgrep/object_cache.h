#ifndef OBJECT_CACHE_H
#define OBJECT_CACHE_H

#include <llvm/ADT/SmallString.h>
#include <llvm/ExecutionEngine/ObjectCache.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/MemoryBuffer.h>
#include <string>

class ICGrepObjectCache : public llvm::ObjectCache {
    public:
        ICGrepObjectCache(const std::string &dir);
        ICGrepObjectCache();
        virtual ~ICGrepObjectCache();

        void notifyObjectCompiled(const llvm::Module *M, llvm::MemoryBufferRef Obj) override;
        std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module* M) override;

    private:
        using Path = llvm::SmallString<256>;
        Path CacheDir;

        bool getCacheFilename(const std::string & ModID, Path & CacheName);
};

#endif
