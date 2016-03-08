#include <string>

#include "object_cache.h"
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>

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
// file exists. CacheDir defaults to "", in which case objects are cached
// alongside their originating bitcodes.
//
ICGrepObjectCache::ICGrepObjectCache(const std::string& CacheDir) : CacheDir(CacheDir) {
    // Add trailing '/' to cache dir if necessary.
    if (!this->CacheDir.empty() &&
            this->CacheDir[this->CacheDir.size() - 1] != '/')
        this->CacheDir += '/';
}

ICGrepObjectCache::~ICGrepObjectCache() {}

void ICGrepObjectCache::notifyObjectCompiled(const Module *M, MemoryBufferRef Obj) {
    const std::string ModuleID = M->getModuleIdentifier();
    std::string CacheName;
    if (!getCacheFilename(ModuleID, CacheName))
        return;
    // TODO use a cleaner, more universal fs API
    if (!CacheDir.empty()) { // Create user-defined cache dir.
        SmallString<128> dir(CacheName);
        sys::path::remove_filename(dir);
        sys::fs::create_directories(Twine(dir));
    }
    std::error_code EC;
    raw_fd_ostream outfile(CacheName, EC, sys::fs::F_None);
    outfile.write(Obj.getBufferStart(), Obj.getBufferSize());
    outfile.close();
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "Cache created: " << CacheName << std::endl;
#endif
}

std::unique_ptr<MemoryBuffer> ICGrepObjectCache::getObject(const Module* M) {
    const std::string ModuleID = M->getModuleIdentifier();
    std::string CacheName;
    if (!getCacheFilename(ModuleID, CacheName))
        return nullptr;
    // Load the object from the cache filename
    ErrorOr<std::unique_ptr<MemoryBuffer>> IRObjectBuffer =
        MemoryBuffer::getFile(CacheName.c_str(), -1, false);
    // If the file isn't there, that's OK.
    if (!IRObjectBuffer)
        return nullptr;
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "Found cached object." << std::endl;
#endif
    // MCJIT will want to write into this buffer, and we don't want that
    // because the file has probably just been mmapped.  Instead we make
    // a copy.  The filed-based buffer will be released when it goes
    // out of scope.
    return MemoryBuffer::getMemBufferCopy(IRObjectBuffer.get()->getBuffer());
}

bool ICGrepObjectCache::getCacheFilename(const std::string &ModID, std::string &CacheName) {
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "ModuleID: " << ModID << std::endl;
#endif
    const static std::string Prefix("grepcode:");
    size_t PrefixLength = Prefix.length();
    if (ModID.substr(0, PrefixLength) != Prefix)
        return false;
/*
//TODO
#if defined(_WIN32)
    // Transform "X:\foo" => "/X\foo" for convenience.
    if (isalpha(CacheSubdir[0]) && CacheSubdir[1] == ':') {
        CacheSubdir[1] = CacheSubdir[0];
        CacheSubdir[0] = '/';
    }
#endif
*/
    CacheName = CacheDir + ModID.substr(PrefixLength) + ".o";
#ifdef OBJECT_CACHE_DEBUG
    std::cerr << "CacheName: " << CacheName << std::endl;
#endif
    return true;
}
