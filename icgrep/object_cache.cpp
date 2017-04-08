#include <string>

#include "object_cache.h"
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>
#include <llvm/IR/Module.h>

#ifdef OBJECT_CACHE_DEBUG
#include <iostream>
#endif

using namespace llvm;

//===----------------------------------------------------------------------===//
// Object cache (based on tools/lli/lli.cpp, LLVM 3.6.1)
//
// This object cache implementation writes cached objects to disk to the
// directory specified by CacheDir, using a filename provided in the module
// descriptor. The cache tries to load a saved object using that path if the
// file exists.
//
ParabixObjectCache::ParabixObjectCache(const std::string &dir): CacheDir(dir) {}

ParabixObjectCache::ParabixObjectCache() {
    // $HOME/.cache/icgrep
    // TODO use path::user_cache_directory once we have llvm >= 3.7.
    sys::path::home_directory(CacheDir);
    std::string Version = PARABIX_VERSION;
    std::string Date = __DATE__;
    std::string Time = __TIME__;
    std::string DateStamp = Date.substr(7) + Date.substr(0,3) + (Date[4] == ' ' ? Date.substr(5,1) : Date.substr(4,2));
    std::string CacheSubDir = "Parabix" + Version + "_" + DateStamp + "@" + Time;
    sys::path::append(CacheDir, ".cache", CacheSubDir);
}

ParabixObjectCache::~ParabixObjectCache() {}

// A new module has been compiled.   If it is cacheable and no conflicting module
// exists, write it out.  
void ParabixObjectCache::notifyObjectCompiled(const Module *M, MemoryBufferRef Obj) {
    const std::string &ModuleID = M->getModuleIdentifier();
    auto f = cachedObjectMap.find(ModuleID);
    if (f!= cachedObjectMap.end()) return;
    Path CacheName(CacheDir);
    if (!getCacheFilename(ModuleID, CacheName)) return;
    if (!CacheDir.empty())      // Re-creating an existing directory is fine.
        sys::fs::create_directories(Twine(CacheDir));
    std::error_code EC;
    raw_fd_ostream outfile(CacheName, EC, sys::fs::F_None);
    outfile.write(Obj.getBufferStart(), Obj.getBufferSize());
    outfile.close();
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "Cache file created: " << CacheName.c_str() << std::endl;
#endif
    auto s = kernelSignatureMap.find(ModuleID);  // Check for a kernel signature.
    if (s != kernelSignatureMap.end()) { 
        if (s->second == ModuleID) return;  // No signature is written when the signature is the ModuleID.
        sys::path::replace_extension(CacheName, ".sig");
        raw_fd_ostream sigfile(CacheName, EC, sys::fs::F_None);
        sigfile << s->second;
        sigfile.close();
#ifdef OBJECT_CACHE_DEBUG
        std::cerr << "Signature file created: " << CacheName.c_str() << std::endl;
#endif
    }
}

bool ParabixObjectCache::loadCachedObjectFile(std::string ModuleID, std::string signature) {
    // Have we already seen this module before.
    auto s = kernelSignatureMap.find(ModuleID);
    if (s!= kernelSignatureMap.end()) {
        if (s->second != signature) {
#ifdef OBJECT_CACHE_DEBUG
            std::cerr << "loadCachedObjectFile:  conflicting signatures for the same moduleID! " << ModuleID << std::endl;
#endif
            return false;
        }
        // A cached entry exists if it has already been loaded.
        return cachedObjectMap.find(ModuleID) != cachedObjectMap.end();
    }
    // Confirm that the module is cacheable.
    Path CachedObjectName(CacheDir);
    if (!getCacheFilename(ModuleID, CachedObjectName)) return false;
    //
    // Save the signature.
    kernelSignatureMap.emplace(ModuleID, signature);
    //
    // Now checkfor a cache file.
    ErrorOr<std::unique_ptr<MemoryBuffer>> KernelObjectBuffer = MemoryBuffer::getFile(CachedObjectName.c_str(), -1, false);
    if (!KernelObjectBuffer) return false;
    //
    if (ModuleID != signature) {
        // Confirm the signature.
        sys::path::replace_extension(CachedObjectName, ".sig");
        ErrorOr<std::unique_ptr<MemoryBuffer>> SignatureBuffer = MemoryBuffer::getFile(CachedObjectName.c_str(), -1, false);
        if (!SignatureBuffer) {
#ifdef OBJECT_CACHE_DEBUG
            std::cerr << "signature file expected but not Found. " << ModuleID << std::endl;
#endif
            return false;
        }
        StringRef loadedSig = SignatureBuffer.get()->getBuffer();
        StringRef computedSig = signature;
        if (!computedSig.equals(loadedSig)) {
#ifdef OBJECT_CACHE_DEBUG
            std::cerr << "computed signature does not match stored signature: " << ModuleID << std::endl;
#endif
            return false;
        }
        // Signature is confirmed.
#ifdef OBJECT_CACHE_DEBUG
        std::cerr << "loadCachedObjectFile: computed signature matches stored signature. " << ModuleID << std::endl;
#endif
    }
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "Found cached object." << CachedObjectName.c_str() << std::endl;
#endif
    // Make a copy so that the JIT engine can freely modify it.
    cachedObjectMap.emplace(ModuleID, std::move(KernelObjectBuffer.get()));
    return true;
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
    const std::string &ModuleID = M->getModuleIdentifier();
    auto f = cachedObjectMap.find(ModuleID);
    if (f == cachedObjectMap.end()) {
        return nullptr;
    }
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "Object retrieved by engine. "<< ModuleID << std::endl;
#endif
    // Return a copy of the buffer, for MCJIT to modify, if necessary.
    return MemoryBuffer::getMemBufferCopy(f->second.get()->getBuffer());
}

bool ParabixObjectCache::getCacheFilename(const std::string &ModID, Path &CacheName) {
    const std::string Prefix("Parabix:");
    size_t PrefixLength = Prefix.length();
    if (ModID.substr(0, PrefixLength) != Prefix)
        return false;
    CacheName = CacheDir;
    sys::path::append(CacheName, ModID.substr(PrefixLength) + ".o");
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "CacheName: " << CacheName.c_str() << std::endl;
#endif
    return true;
}
