#include "object_cache.h"
#include "object_cache_util.hpp"
#include <kernels/kernel.h>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/IR/Metadata.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>
#include <llvm/Support/Debug.h>
#include <llvm/IR/Module.h>
#include <toolchain/toolchain.h>
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/IR/Verifier.h>
#include <boost/lexical_cast.hpp>

using namespace llvm;
using namespace boost;

using Path = ParabixObjectCache::Path;

std::unique_ptr<ParabixObjectCache> ParabixObjectCache::mInstance;

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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getSignature
 ** ------------------------------------------------------------------------------------------------------------- */
const MDString * getSignature(const llvm::Module * const M) {
    NamedMDNode * const sig = M->getNamedMetadata(SIGNATURE);
    if (sig) {
        assert ("empty metadata node" && sig->getNumOperands() == 1);
        assert ("no signature payload" && sig->getOperand(0)->getNumOperands() == 1);
        return dyn_cast<MDString>(sig->getOperand(0)->getOperand(0));
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadCachedObjectFile
 ** ------------------------------------------------------------------------------------------------------------- */
bool ParabixObjectCache::loadCachedObjectFile(const std::unique_ptr<kernel::KernelBuilder> & idb, kernel::Kernel * const kernel) {
    if (LLVM_LIKELY(kernel->isCachable())) {
        assert (kernel->getModule() == nullptr);
        const auto moduleId = kernel->getCacheName(idb);

        // TODO: To enable the quick lookup of previously cached objects, I need to reclaim ownership
        // of the modules from the JIT engine before it destroys them.

//        // Have we already seen this module before?
//        const auto f = mCachedObject.find(moduleId);
//        if (LLVM_UNLIKELY(f != mCachedObject.end())) {
//            Module * const m = f->second.first; assert (m);
//            kernel->setModule(m);
//            kernel->prepareCachedKernel(idb);
//            return true;
//        }

        // No, check for an existing cache file.
        Path fileName(mCachePath);
        sys::path::append(fileName, CACHE_PREFIX);
        fileName.append(moduleId);
        fileName.append("." KERNEL_FILE_EXTENSION);

        auto kernelBuffer = MemoryBuffer::getFile(fileName, -1, false);
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
                sys::path::replace_extension(fileName, "." OBJECT_FILE_EXTENSION);
                auto objectBuffer = MemoryBuffer::getFile(fileName.c_str(), -1, false);
                if (LLVM_LIKELY(objectBuffer)) {
                    Module * const m = M.release();
                    // defaults to <path>/<moduleId>.kernel
                    m->setModuleIdentifier(moduleId);
                    kernel->setModule(m);
                    kernel->prepareCachedKernel(idb);
                    mCachedObject.emplace(moduleId, std::make_pair(m, std::move(objectBuffer.get())));
                    // update the modified time of the .kernel, .o and .sig files
                    const auto access_time = currentTime();
                    fs::last_write_time(fileName.c_str(), access_time);
                    sys::path::replace_extension(fileName, "." KERNEL_FILE_EXTENSION);
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief notifyObjectCompiled
 *
 * A new module has been compiled. If it is cacheable and no conflicting module exists, write it out.
 ** ------------------------------------------------------------------------------------------------------------- */
void ParabixObjectCache::notifyObjectCompiled(const Module * M, MemoryBufferRef Obj) {
    if (LLVM_LIKELY(M->getNamedMetadata(CACHEABLE))) {

        const auto moduleId = M->getModuleIdentifier();
        Path objectName(mCachePath);
        sys::path::append(objectName, CACHE_PREFIX);
        objectName.append(moduleId);
        objectName.append("." OBJECT_FILE_EXTENSION);

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

        sys::path::replace_extension(objectName, "." KERNEL_FILE_EXTENSION);
        raw_fd_ostream kernelFile(objectName.str(), EC, sys::fs::F_None);
        WriteBitcodeToFile(H.get(), kernelFile);
        kernelFile.close();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getObject
 ** ------------------------------------------------------------------------------------------------------------- */
std::unique_ptr<MemoryBuffer> ParabixObjectCache::getObject(const Module * module) {
    const auto f = mCachedObject.find(module->getModuleIdentifier());
    if (f == mCachedObject.end()) {
        return nullptr;
    }
    // Return a copy of the buffer, for MCJIT to modify, if necessary.
    return MemoryBuffer::getMemBufferCopy(f->second.second.get()->getBuffer());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
bool ParabixObjectCache::checkForCachedKernel(const std::unique_ptr<kernel::KernelBuilder> & b, kernel::Kernel * const kernel) noexcept {
    return mInstance.get() && mInstance->loadCachedObjectFile(b, kernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCacheCleanUp
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool ParabixObjectCache::requiresCacheCleanUp() noexcept {
    return FileLock(fs::path{mCachePath.str()}).locked();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiateCacheCleanUp
 ** ------------------------------------------------------------------------------------------------------------- */
void ParabixObjectCache::initiateCacheCleanUp() noexcept {
    if (LLVM_UNLIKELY(requiresCacheCleanUp())) {
        // syslog?
        if (fork() == 0) {
            char * const cachePath = const_cast<char *>(mCachePath.c_str());
            char * args[3] = {const_cast<char *>(CACHE_JANITOR_FILE_NAME), cachePath, nullptr};
            fs::path path(codegen::ProgramName);
            path.remove_filename().append(CACHE_JANITOR_FILE_NAME);
            if (execvp(const_cast<char *>(path.c_str()), args) < 0) {
                // syslog?
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDefaultCachePath
 ** ------------------------------------------------------------------------------------------------------------- */
inline void getDefaultCachePath(Path & configPath) {
    // default: $HOME/.cache/parabix/
    sys::path::home_directory(configPath);
    sys::path::append(configPath, ".cache", "parabix");
}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getConfigPath
 ** ------------------------------------------------------------------------------------------------------------- */
inline Path getConfigPath() {
    // $HOME/.config/parabix/cache.cfg
    Path configPath;
    sys::path::home_directory(configPath);
    sys::path::append(configPath, ".config", "parabix");
    sys::fs::create_directories(configPath);
    sys::path::append(configPath, "cache.cfg");
    return configPath;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadCacheSettings
 ** ------------------------------------------------------------------------------------------------------------- */
inline size_t parseInt(const StringRef & str, const StringRef & label) {
    try {
        return lexical_cast<size_t>(str.data(), str.size());
    } catch(const bad_lexical_cast &) {
        errs() << "configuration for " << label << " must be an integer";
        exit(-1);
    }
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadCacheSettings
 ** ------------------------------------------------------------------------------------------------------------- */
inline void ParabixObjectCache::loadCacheSettings() noexcept {
    getDefaultCachePath(mCachePath);
    #if 0

    const auto configPath = getConfigPath();
    auto configFile = MemoryBuffer::getFile(configPath);

    // default: $HOME/.cache/parabix/
    sys::path::home_directory(mCachePath);
    sys::path::append(mCachePath, ".cache", "parabix");
    // default: 1 week
    mCacheExpirationDelay = CACHE_ENTRY_EXPIRY_PERIOD;

    if (LLVM_UNLIKELY(!!configFile)) {
        const StringRef config = (*configFile)->getBuffer();
        #define ASCII_WHITESPACE " \f\n\r\t\v"
        #define ASCII_WHITESPACE_OR_EQUALS (ASCII_WHITESPACE "+")
        size_t nameStart = 0;
        for (;;) {

            const auto nameEnd = config.find_first_of(ASCII_WHITESPACE_OR_EQUALS, nameStart);
            if (nameEnd == StringRef::npos) break;
            const auto afterEquals = config.find_first_of('=', nameEnd) + 1;
            if (LLVM_UNLIKELY(afterEquals == StringRef::npos)) break;
            const auto valueStart = config.find_first_not_of(ASCII_WHITESPACE, afterEquals);
            if (LLVM_UNLIKELY(valueStart == StringRef::npos)) break;
            const auto valueEnd = config.find_first_of(ASCII_WHITESPACE, valueStart);
            if (LLVM_UNLIKELY(valueEnd == StringRef::npos)) break;
            const auto name = config.slice(nameStart, nameEnd);
            const auto value = config.slice(valueStart, valueEnd);

            if (name.equals_lower("cachepath")) {
                mCachePath.assign(value);
            } else if (name.equals_lower("cachedayslimit")) {
                mCacheExpirationDelay = parseInt(value, "cachedayslimit");
            }
            // get the next name start
            nameStart = config.find_first_not_of(ASCII_WHITESPACE, valueEnd + 1);
        }
    }
    #endif
    sys::fs::create_directories(mCachePath);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief saveCachePath
 ** ------------------------------------------------------------------------------------------------------------- */
inline void ParabixObjectCache::saveCacheSettings() noexcept {


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCacheSystems
 ** ------------------------------------------------------------------------------------------------------------- */
void ParabixObjectCache::initializeCacheSystems() noexcept {
    if (LLVM_LIKELY(mInstance.get() == nullptr && codegen::EnableObjectCache)) {
        mInstance.reset(new ParabixObjectCache());
    }
}

ParabixObjectCache::ParabixObjectCache() {
    loadCacheSettings();
    initiateCacheCleanUp();
}
