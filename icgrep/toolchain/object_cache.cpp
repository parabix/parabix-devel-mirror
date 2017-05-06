#include "object_cache.h"
#include <kernels/kernel.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>
#include <llvm/IR/Module.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>
#include <ctime>

using namespace llvm;

//===----------------------------------------------------------------------===//
// Object cache (based on tools/lli/lli.cpp, LLVM 3.6.1)
//
// This object cache implementation writes cached objects to disk to the
// directory specified by CacheDir, using a filename provided in the module
// descriptor. The cache tries to load a saved object using that path if the
// file exists.
//

#define MONTH_1 \
    ((__DATE__ [0] == 'O' || __DATE__ [0] == 'N' || __DATE__ [0] == 'D') ? '1' : '0')
#define MONTH_2 \
    (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? '1' : '6') \
    : __DATE__ [2] == 'b' ? '2' \
    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? '3' : '4') \
    : __DATE__ [2] == 'y' ? '5' \
    : __DATE__ [2] == 'l' ? '7' \
    : __DATE__ [2] == 'g' ? '8' \
    : __DATE__ [2] == 'p' ? '9' \
    : __DATE__ [2] == 't' ? '0' \
    : __DATE__ [2] == 'v' ? '1' : '2')
#define DAY_1 (__DATE__[4] == ' ' ? '0' : __DATE__[4])
#define DAY_2 (__DATE__[5])
#define YEAR_1 (__DATE__[9])
#define YEAR_2 (__DATE__[10])
#define HOUR_1 (__TIME__[0])
#define HOUR_2 (__TIME__[1])
#define MINUTE_1 (__TIME__[3])
#define MINUTE_2 (__TIME__[4])
#define SECOND_1 (__TIME__[6])
#define SECOND_2 (__TIME__[7])

const static auto CACHE_PREFIX = PARABIX_VERSION +
                          std::string{'@',
                          MONTH_1, MONTH_2, DAY_1, DAY_2, YEAR_1, YEAR_2,
                          HOUR_1, HOUR_2, MINUTE_1, MINUTE_2, SECOND_1, SECOND_2,
                          '_'};

bool ParabixObjectCache::loadCachedObjectFile(kernel::Kernel * const kernel) {
    if (LLVM_LIKELY(kernel->isCachable())) {

        Module * const module = kernel->getModule();
        assert ("kernel module cannot be null!" && module);
        const auto moduleId = module->getModuleIdentifier();

        // Have we already seen this module before?
        if (LLVM_UNLIKELY(mCachedObjectMap.count(moduleId) != 0)) {
            const auto f = mKernelSignatureMap.find(moduleId);
            if (f == mKernelSignatureMap.end()) {
                return kernel->moduleIDisSignature();
            } else if (kernel->moduleIDisSignature() || (kernel->makeSignature() != f->second)) {
                return false;
            }
            return true;
        }

        // No, check for an existing cache file.
        Path objectName(mCachePath);
        sys::path::append(objectName, CACHE_PREFIX);
        objectName.append(moduleId);
        objectName.append(".o");

        auto objectBuffer = MemoryBuffer::getFile(objectName.c_str(), -1, false);
        if (objectBuffer) {
            if (!kernel->moduleIDisSignature()) {
                sys::path::replace_extension(objectName, ".sig");
                const auto signatureBuffer = MemoryBuffer::getFile(objectName.c_str(), -1, false);
                if (signatureBuffer) {
                    const StringRef loadedSig = signatureBuffer.get()->getBuffer();
                    if (!loadedSig.equals(kernel->makeSignature())) {
                        return false;
                    }
                } else {
                    report_fatal_error("signature file expected but not found: " + moduleId);
                    return false;
                }
            }
            // updae the modified time of the file then add it to our cache
            boost::filesystem::last_write_time(objectName.c_str(), time(0));
            mCachedObjectMap.emplace(moduleId, std::move(objectBuffer.get()));
            return true;
        } else if (!kernel->moduleIDisSignature()) {
            mKernelSignatureMap.emplace(moduleId, kernel->makeSignature());
        }
    }
    return false;
}

// A new module has been compiled. If it is cacheable and no conflicting module
// exists, write it out.
void ParabixObjectCache::notifyObjectCompiled(const Module * M, MemoryBufferRef Obj) {
    const auto moduleId = M->getModuleIdentifier();
    if (mCachedObjectMap.count(moduleId) == 0) {

        Path objectName(mCachePath);
        sys::path::append(objectName, CACHE_PREFIX);
        objectName.append(moduleId);
        objectName.append(".o");

        if (LLVM_LIKELY(!mCachePath.empty())) {
            sys::fs::create_directories(Twine(mCachePath));
        }

        std::error_code EC;
        raw_fd_ostream outfile(objectName, EC, sys::fs::F_None);
        outfile.write(Obj.getBufferStart(), Obj.getBufferSize());
        outfile.close();

        // If this kernel has a signature, write it.
        const auto sig = mKernelSignatureMap.find(moduleId);
        if (LLVM_UNLIKELY(sig != mKernelSignatureMap.end())) {
            sys::path::replace_extension(objectName, ".sig");
            raw_fd_ostream sigfile(objectName, EC, sys::fs::F_None);
            sigfile << sig->second;
            sigfile.close();
        }
    }
}

/*  May need this.

void ParabixObjectCache::removeCacheFile(std::string ModuleID) {
    Path CacheName(CacheDir);
    if (!getCacheFilename(ModuleID, CacheName)) return;
    sys::fs::remove(CacheName);
    // Also remove a signature file, if present.
    sys::path::replace_extension(CacheName, ".sig");
    sys::fs::remove(CacheName);
}
*/

std::unique_ptr<MemoryBuffer> ParabixObjectCache::getObject(const Module* M) {
    auto f = mCachedObjectMap.find(M->getModuleIdentifier());
    if (f == mCachedObjectMap.end()) {
        return nullptr;
    }
    // Return a copy of the buffer, for MCJIT to modify, if necessary.
    return MemoryBuffer::getMemBufferCopy(f->second.get()->getBuffer());
}

inline ParabixObjectCache::Path ParabixObjectCache::getDefaultPath() {
    // $HOME/.cache/parabix/
    Path cachePath;
    // TODO use path::user_cache_directory once we have llvm >= 3.7.
    sys::path::home_directory(cachePath);
    sys::path::append(cachePath, ".cache", "parabix");
    return cachePath;
}

ParabixObjectCache::ParabixObjectCache()
: mCachePath(std::move(getDefaultPath())) {

}

ParabixObjectCache::ParabixObjectCache(const std::string & dir)
: mCachePath(dir) {

}
