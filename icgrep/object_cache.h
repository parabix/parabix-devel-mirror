#ifndef OBJECT_CACHE_H
#define OBJECT_CACHE_H

#include <string>

#include <llvm/IR/Module.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/ExecutionEngine/ObjectCache.h>
#include <llvm/ADT/SmallString.h>

class ICGrepObjectCache : public llvm::ObjectCache {
    public:
        ICGrepObjectCache(const std::string &dir);
        ICGrepObjectCache();
        virtual ~ICGrepObjectCache();

        void notifyObjectCompiled(const llvm::Module *M, llvm::MemoryBufferRef Obj) override;
        std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module* M) override;

    private:
        const static size_t mPathInitLength = 256;
        typedef llvm::SmallString<mPathInitLength> Path;
        Path CacheDir;

        bool getCacheFilename(const std::string &ModID, Path &CacheName);
};

#endif
