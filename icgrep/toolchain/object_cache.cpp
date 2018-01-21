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
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/IR/Verifier.h>
#include <ctime>

using namespace llvm;
namespace fs = boost::filesystem;

#ifdef NDEBUG
#define CACHE_ENTRY_MAX_HOURS (24 * codegen::CacheDaysLimit)
#else
#define CACHE_ENTRY_MAX_HOURS (1)
#endif

#define SECONDS_PER_HOUR (3600)
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
        return dyn_cast<MDString>(sig->getOperand(0)->getOperand(0));
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
        Path fileName(mCachePath);
        sys::path::append(fileName, CACHE_PREFIX);
        fileName.append(moduleId);
        fileName.append(".kernel");

        auto kernelBuffer = MemoryBuffer::getFile(fileName.c_str(), -1, false);
        if (kernelBuffer) {
            #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
            auto loadedFile = getLazyBitcodeModule(std::move(kernelBuffer.get()), idb->getContext());
            #else
            auto loadedFile = getOwningLazyBitcodeModule(std::move(kernelBuffer.get()), idb->getContext());
            #endif
            // if there was no error when parsing the bitcode
            if (LLVM_LIKELY(loadedFile)) {
                std::unique_ptr<Module> M(std::move(loadedFile.get()));
                if (kernel->hasSignature()) {
                    const MDString * const sig = getSignature(M.get());
                    assert ("signature is missing from kernel file: possible module naming conflict or change in the LLVM metadata storage policy?" && sig);
                    if (LLVM_UNLIKELY(sig == nullptr || !sig->getString().equals(kernel->makeSignature(idb)))) {
                        goto invalid;
                    }
                }
                sys::path::replace_extension(fileName, ".o");
                auto objectBuffer = MemoryBuffer::getFile(fileName.c_str(), -1, false);
                if (LLVM_LIKELY(objectBuffer)) {
                    Module * const m = M.release();
                    // defaults to <path>/<moduleId>.kernel
                    m->setModuleIdentifier(moduleId);
                    kernel->setModule(m);
                    kernel->prepareCachedKernel(idb);
                    mCachedObject.emplace(moduleId, std::make_pair(m, std::move(objectBuffer.get())));
                    // update the modified time of the .kernel, .o and .sig files
                    time_t access_time = time(0);
                    fs::last_write_time(fileName.c_str(), access_time);
                    sys::path::replace_extension(fileName, ".kernel");
                    fs::last_write_time(fileName.c_str(), access_time);
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

        // and kernel prototype header
        std::unique_ptr<Module> H(new Module(M->getModuleIdentifier(), M->getContext()));
        for (const Function & f : M->getFunctionList()) {
            if (f.hasExternalLinkage() && !f.empty()) {
                Function::Create(f.getFunctionType(), Function::ExternalLinkage, f.getName(), H.get());
            }
        }

        // then the signature (if one exists)
        const MDString * const sig = getSignature(M);
        if (sig) {
            NamedMDNode * const md = H->getOrInsertNamedMetadata(SIGNATURE);
            assert (md->getNumOperands() == 0);
            MDString * const sigCopy = MDString::get(H->getContext(), sig->getString());
            md->addOperand(MDNode::get(H->getContext(), {sigCopy}));
        }

        sys::path::replace_extension(objectName, ".kernel");
        raw_fd_ostream kernelFile(objectName.str(), EC, sys::fs::F_None);
        WriteBitcodeToFile(H.get(), kernelFile);
        kernelFile.close();
    }
}

void ParabixObjectCache::performIncrementalCacheCleanupStep() {
    mCleanupMutex.lock();
    if (LLVM_UNLIKELY(mCleanupIterator == fs::directory_iterator())) {
        mCleanupMutex.unlock();
    } else {
        const auto e = mCleanupIterator->path();
        mCleanupIterator++;
        mCleanupMutex.unlock();

        // Simple clean-up policy: files that haven't been touched by the
        // driver in MaxCacheEntryHours are deleted.
        // TODO: possibly incrementally manage by size and/or total file count.
        // TODO: possibly determine total filecount and set items per clean up step based on
        // filecount
        if (fs::is_regular_file(e)) {
            const auto age = std::time(nullptr) - fs::last_write_time(e);
            if (age > (CACHE_ENTRY_MAX_HOURS * SECONDS_PER_HOUR)) {
                fs::remove(e);
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

ParabixObjectCache::ParabixObjectCache(const StringRef dir)
: mCachePath(dir) {
    fs::path p(mCachePath.str());
    if (LLVM_LIKELY(!mCachePath.empty())) {
        sys::fs::create_directories(mCachePath);
    }
    mCleanupIterator = fs::directory_iterator(p);
}

inline ParabixObjectCache::Path getDefaultPath() {
    // $HOME/.cache/parabix/
    ParabixObjectCache::Path cachePath;
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(3, 7, 0)
    sys::path::user_cache_directory(cachePath, "parabix");
#else
    sys::path::home_directory(cachePath);
    sys::path::append(cachePath, ".cache", "parabix");
#endif
    return cachePath;
}

ParabixObjectCache::ParabixObjectCache()
: ParabixObjectCache(getDefaultPath()) {

}


