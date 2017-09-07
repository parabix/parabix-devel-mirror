#include "object_cache.h"
#include <kernels/kernel.h>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/IR/Metadata.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>
#include <llvm/IR/Module.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/container/flat_set.hpp>
#include <llvm/Bitcode/ReaderWriter.h>
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

const static boost::uintmax_t CACHE_SIZE_LIMIT = 50 * 1024 * 1024;

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
                auto loadedFile = getLazyBitcodeModule(std::move(kernelBuffer.get()), idb->getContext());
                if (*loadedFile) {
                    Module * const m = loadedFile.get().release(); assert (m);
                    // defaults to <path>/<moduleId>.kernel
                    m->setModuleIdentifier(moduleId);
                    kernel->setModule(m);
                    kernel->prepareCachedKernel(idb);                    
                    mCachedObject.emplace(moduleId, std::make_pair(m, std::move(objectBuffer.get())));
                    // update the modified time of the object file
                    sys::path::replace_extension(objectName, ".o");
                    boost::filesystem::last_write_time(objectName.c_str(), time(0));
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

        if (LLVM_LIKELY(!mCachePath.empty())) {
            sys::fs::create_directories(Twine(mCachePath));
        }

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
        WriteBitcodeToFile(header.get(), kernelFile, false, false);
        kernelFile.close();
    }
}

void ParabixObjectCache::cleanUpObjectCacheFiles() {

    using namespace boost::filesystem;
    using ObjectFile = std::pair<std::time_t, path>;

    path cachePath(mCachePath.str());
    if (LLVM_LIKELY(is_directory(cachePath))) {
        std::vector<ObjectFile> files;
        for(const directory_entry & entry : boost::make_iterator_range(directory_iterator(cachePath), {})) {
            const auto path = entry.path();;
            if (LLVM_LIKELY(is_regular_file(path) && path.has_extension() && path.extension().compare(".o") == 0)) {
                files.emplace_back(last_write_time(path), path.filename());
            }
        }
        // sort the files in decending order of last modified (datetime) then file name
        std::sort(files.begin(), files.end(), std::greater<ObjectFile>());
        boost::uintmax_t cacheSize = 0;
        for(const ObjectFile & entry : files) {
            auto objectPath = cachePath / std::get<1>(entry);
            if (LLVM_LIKELY(exists(objectPath))) {
                const auto size = file_size(objectPath);
                if ((cacheSize + size) < CACHE_SIZE_LIMIT) {
                    cacheSize += size;
                } else {
                    remove(objectPath);
                    objectPath.replace_extension("sig");
                    remove(objectPath);
                    objectPath.replace_extension("kernel");
                    remove(objectPath);
                }
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

inline ParabixObjectCache::Path ParabixObjectCache::getDefaultPath() {
    // $HOME/.cache/parabix/
    Path cachePath;
    #ifndef USE_LLVM_3_6
    sys::path::user_cache_directory(cachePath, "parabix");
    #else
    sys::path::home_directory(cachePath);
    sys::path::append(cachePath, ".cache", "parabix");
    #endif
    return cachePath;
}

ParabixObjectCache::ParabixObjectCache()
: mCachePath(getDefaultPath()) {

}

ParabixObjectCache::ParabixObjectCache(const std::string dir)
: mCachePath(dir) {

}
