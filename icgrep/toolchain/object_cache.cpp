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
ParabixObjectCache::ParabixObjectCache(const std::string &dir): mCachePath(dir) {}

ParabixObjectCache::ParabixObjectCache() {
    // $HOME/.cache/icgrep
    // TODO use path::user_cache_directory once we have llvm >= 3.7.
    sys::path::home_directory(mCachePath);
    sys::path::append(mCachePath, ".cache", "parabix");

    const std::string Version = PARABIX_VERSION;
    const std::string Date = __DATE__;
    const std::string Time = __TIME__;
    const std::string DateStamp = Date.substr(7) + Date.substr(0, 3) + (Date[4] == ' ' ? Date.substr(5, 1) : Date.substr(4, 2));
    mCachePrefix = Version + "_" + DateStamp + "@" + Time;
}

bool ParabixObjectCache::loadCachedObjectFile(kernel::KernelBuilder * const kernel) {
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
        sys::path::append(objectName, mCachePrefix + moduleId + ".o");
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
        sys::path::append(objectName, mCachePrefix + moduleId + ".o");

        if (LLVM_LIKELY(!mCachePath.empty())) {
            sys::fs::create_directories(Twine(mCachePath));
        }

        std::error_code EC;
        raw_fd_ostream outfile(objectName, EC, sys::fs::F_None);
        outfile.write(Obj.getBufferStart(), Obj.getBufferSize());
        outfile.close();

        // Check for a kernel signature.
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

