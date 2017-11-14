#include "toolchain.h"
#include "object_cache.h"
#include <kernels/kernel.h>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/IR/Metadata.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>
#include <llvm/Support/Debug.h> 
#include <llvm/IR/Module.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/container/flat_set.hpp>
#if LLVM_VERSION_INTEGER < LLVM_4_0_0
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/IR/Verifier.h>
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

const static auto CACHEABLE = "cacheable";

const static auto SIGNATURE = "signature";

const MDString * getSignature(const llvm::Module * const M) {
    NamedMDNode * const sig = M->getNamedMetadata(SIGNATURE);
    if (sig) {
        assert ("empty metadata node" && sig->getNumOperands() == 1);
        assert ("no signature payload" && sig->getOperand(0)->getNumOperands() == 1);
        return cast<MDString>(sig->getOperand(0)->getOperand(0));
    }
    return nullptr;
}

bool ParabixObjectCache::loadCachedObjectFile(const std::unique_ptr<kernel::KernelBuilder> & idb, kernel::Kernel * const kernel) {
    if (LLVM_LIKELY(kernel->isCachable())) {
        assert (kernel->getModule() == nullptr);
        const auto moduleId = kernel->getCacheName(idb);

        // Have we already seen this module before?
        const auto f = mCachedObject.find(moduleId);
        if (LLVM_UNLIKELY(f != mCachedObject.end())) {
            Module * const m = f->second.first; assert (m);
            kernel->setModule(m);
            kernel->prepareCachedKernel(idb);
            return true;
        }

        // No, check for an existing cache file.
        Path objectName(mCachePath);
        sys::path::append(objectName, CACHE_PREFIX);
        objectName.append(moduleId);
        objectName.append(".o");

        auto objectBuffer = MemoryBuffer::getFile(objectName.c_str(), -1, false);
        if (objectBuffer) {
            if (kernel->hasSignature()) {
                sys::path::replace_extension(objectName, ".sig");
                const auto signatureBuffer = MemoryBuffer::getFile(objectName.c_str(), -1, false);
                if (signatureBuffer) {
                    const StringRef loadedSig = signatureBuffer.get()->getBuffer();
                    if (LLVM_UNLIKELY(!loadedSig.equals(kernel->makeSignature(idb)))) {
                        goto invalid;
                    }
                } else {
                    
                    report_fatal_error("signature file expected but not found: " + moduleId);
                }                
            }
            sys::path::replace_extension(objectName, ".kernel");
            auto kernelBuffer = MemoryBuffer::getFile(objectName.c_str(), -1, false);
            if (*kernelBuffer) {
                //MemoryBuffer * kb = kernelBuffer.get().release();
                //auto loadedFile = parseBitcodeFile(kb->getMemBufferRef(), mContext);
#if LLVM_VERSION_INTEGER < LLVM_4_0_0
                auto loadedFile = getLazyBitcodeModule(std::move(kernelBuffer.get()), idb->getContext());
#else
                auto loadedFile = getOwningLazyBitcodeModule(std::move(kernelBuffer.get()), idb->getContext());
#endif
                if (*loadedFile) {
                    Module * const m = loadedFile.get().release(); assert (m);
                    // defaults to <path>/<moduleId>.kernel
                    m->setModuleIdentifier(moduleId);
                    kernel->setModule(m);
                    kernel->prepareCachedKernel(idb);                    
                    mCachedObject.emplace(moduleId, std::make_pair(m, std::move(objectBuffer.get())));
                    // update the modified time of the .kernel, .o and .sig files
                    time_t access_time = time(0);
                    boost::filesystem::last_write_time(objectName.c_str(), access_time);
                    sys::path::replace_extension(objectName, ".o");
                    boost::filesystem::last_write_time(objectName.c_str(), access_time);
                    if (kernel->hasSignature()) {
                        sys::path::replace_extension(objectName, ".sig");
                        boost::filesystem::last_write_time(objectName.c_str(), access_time);
                    }
                    return true;
                }
            }
        }

invalid:

        Module * const module = kernel->setModule(new Module(moduleId, idb->getContext()));
        // mark this module as cachable
        module->getOrInsertNamedMetadata(CACHEABLE);
        // if this module has a signature, add it to the metadata
        if (kernel->hasSignature()) {
            NamedMDNode * const md = module->getOrInsertNamedMetadata(SIGNATURE);
            assert (md->getNumOperands() == 0);
            MDString * const sig = MDString::get(module->getContext(), kernel->makeSignature(idb));
            md->addOperand(MDNode::get(module->getContext(), {sig}));
        }
    }
    return false;
}

// A new module has been compiled. If it is cacheable and no conflicting module
// exists, write it out.
void ParabixObjectCache::notifyObjectCompiled(const Module * M, MemoryBufferRef Obj) {
    if (LLVM_LIKELY(M->getNamedMetadata(CACHEABLE))) {
        const auto moduleId = M->getModuleIdentifier();
        Path objectName(mCachePath);
        sys::path::append(objectName, CACHE_PREFIX);
        objectName.append(moduleId);
        objectName.append(".o");

        // Write the object code
        std::error_code EC;
        raw_fd_ostream objFile(objectName, EC, sys::fs::F_None);
        objFile.write(Obj.getBufferStart(), Obj.getBufferSize());
        objFile.close();

        // then the signature (if one exists)
        const MDString * const sig = getSignature(M);
        if (sig) {
            sys::path::replace_extension(objectName, ".sig");
            raw_fd_ostream sigfile(objectName, EC, sys::fs::F_None);
            sigfile << sig->getString();
            sigfile.close();
        }

        // and finally kernel prototype header.
        std::unique_ptr<Module> header(new Module(M->getModuleIdentifier(), M->getContext()));
        for (const Function & f : M->getFunctionList()) {
            if (f.hasExternalLinkage() && !f.empty()) {
                Function::Create(f.getFunctionType(), Function::ExternalLinkage, f.getName(), header.get());
            }
        }

        sys::path::replace_extension(objectName, ".kernel");
        raw_fd_ostream kernelFile(objectName.str(), EC, sys::fs::F_None);
        WriteBitcodeToFile(header.get(), kernelFile);
        kernelFile.close();
    }
}

void ParabixObjectCache::performIncrementalCacheCleanupStep() {
    if (mCacheCleanupIterator != boost::filesystem::directory_iterator()) {
        auto & e = mCacheCleanupIterator->path();
        mCacheCleanupIterator++;
        // Simple clean-up policy: files that haven't been touched by the
        // driver in MaxCacheEntryHours are deleted.
        // TODO: possibly incrementally manage by size and/or total file count.
        // TODO: possibly determine total filecount and set items per clean up step based on
        // filecount
        if (boost::filesystem::is_regular_file(e)) {
            auto age = std::time(nullptr) - boost::filesystem::last_write_time(e);
            if (age > mCacheEntryMaxHours * 3600 /* secs/hour*/ ) {
                boost::filesystem::remove(e);
                errs() << e.string() << " removed.\n";
            }
        }
    }
}

std::unique_ptr<MemoryBuffer> ParabixObjectCache::getObject(const Module * module) {
    const auto f = mCachedObject.find(module->getModuleIdentifier());
    if (f == mCachedObject.end()) {
        return nullptr;
    }
    // Return a copy of the buffer, for MCJIT to modify, if necessary.
    return MemoryBuffer::getMemBufferCopy(f->second.second.get()->getBuffer());
}

inline std::string ParabixObjectCache::getDefaultPath() {
    // $HOME/.cache/parabix/
    Path cachePath;
#if LLVM_VERSION_INTEGER < LLVM_3_7_0
    sys::path::user_cache_directory(cachePath, "parabix");
#else
    sys::path::home_directory(cachePath);
    sys::path::append(cachePath, ".cache", "parabix");
#endif
    return cachePath.str();
}

ParabixObjectCache::ParabixObjectCache(const std::string dir)
: mCachePath(dir) {
    boost::filesystem::path p(mCachePath.str());
    if (LLVM_LIKELY(!mCachePath.empty())) {
        sys::fs::create_directories(Twine(mCachePath));
    }
    boost::filesystem::directory_iterator it(p);
    mCacheCleanupIterator = it;
    mCacheEntryMaxHours = CACHE_ENTRY_MAX_HOURS;
}

ParabixObjectCache::ParabixObjectCache()
: ParabixObjectCache(getDefaultPath()) {
}


