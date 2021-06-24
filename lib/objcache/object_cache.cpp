#include <objcache/object_cache.h>

#include <objcache/object_cache_util.hpp>
#include <kernel/core/kernel.h>
#include <kernel/core/kernel_builder.h>
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
#include <system_error>

using namespace llvm;
using namespace boost;



using Path = ParabixObjectCache::Path;

bool ParabixObjectCache::mStartedCacheCleanupDaemon = false;

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
        assert ("empty metadata node" && sig->getNumOperands() > 0);
        assert ("metadata should contain precisely one node" && sig->getNumOperands() == 1);
        assert ("no signature payload" && sig->getOperand(0)->getNumOperands() == 1);
        return cast<MDString>(sig->getOperand(0)->getOperand(0));
    }
    return nullptr;
}

inline bool isNonMatchingSignature(const MDString * const received, const StringRef expected) {
    return !expected.equals(received->getString());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadCachedObjectFile
 ** ------------------------------------------------------------------------------------------------------------- */
CacheObjectResult ParabixObjectCache::loadCachedObjectFile(BuilderRef b, kernel::Kernel * const kernel) noexcept {

    assert (kernel->getModule() == nullptr);

    // Have we already seen this signature before? if so, we can safely assume that the ExecutionEngine
    // will have a compiled module for this kernel when we execute the pipeline.
    const auto signature = kernel->getSignature();
    const auto f = mKnownSignatures.find(std::string(signature));
    if (LLVM_UNLIKELY(f != mKnownSignatures.end())) {
        if (LLVM_UNLIKELY(codegen::TraceObjectCache)) {
            const auto moduleId = kernel->makeCacheName(b);
            errs() << "Already compiled: " << moduleId << KERNEL_FILE_EXTENSION << "\n";
        }        
        kernel->setModule(f->second);
        return CacheObjectResult::COMPILED;
    }

    if (LLVM_LIKELY(kernel->isCachable())) {
        Path fileName(mCachePath);
        const auto moduleId = kernel->makeCacheName(b);
        sys::path::append(fileName, CACHE_PREFIX);
        fileName.append(moduleId);
        fileName.append(KERNEL_FILE_EXTENSION);
        auto kernelBuffer = MemoryBuffer::getFile(fileName, -1, false);
        if (kernelBuffer) {
            #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
            auto loadedFile = getLazyBitcodeModule(std::move(kernelBuffer.get()), b->getContext());
            #else
            auto loadedFile = getOwningLazyBitcodeModule(std::move(kernelBuffer.get()), b->getContext());
            #endif
            // if there was no error when parsing the bitcode
            if (LLVM_LIKELY(loadedFile)) {
                std::unique_ptr<Module> M(std::move(loadedFile.get()));
                if (LLVM_UNLIKELY(kernel->hasSignature())) {
                    const MDString * const sig = getSignature(M.get());
                    assert ("signature is missing from kernel file: possible module naming conflict or change in the LLVM metadata storage policy?" && sig);
                    if (LLVM_UNLIKELY(isNonMatchingSignature(sig, signature))) {
                        if (LLVM_UNLIKELY(codegen::TraceObjectCache)) {
                            errs() << "Mismatched signature in cache file: " << moduleId << KERNEL_FILE_EXTENSION << "\n"
                                      "Expected: " << signature << "\n"
                                      "Loaded:   " << sig->getString() << "\n";
                        }
                        goto invalid;
                    }
                }
                sys::path::replace_extension(fileName, OBJECT_FILE_EXTENSION);
                auto objectBuffer = MemoryBuffer::getFile(fileName.c_str(), -1, false);
                if (LLVM_LIKELY(objectBuffer)) {
                    Module * const m = M.release();
                    assert ("object cache file returned null module?" && m);
                    // defaults to <path>/<moduleId>.kernel
                    m->setModuleIdentifier(moduleId);
                    b->setModule(m);
                    kernel->loadCachedKernel(b);
                    mCachedObject.emplace(moduleId, std::move(objectBuffer.get()));
                    mKnownSignatures.emplace(signature, m);
                    // update the modified time of the .o and .kernel files
                    const auto access_time = currentTime();
                    fs::last_write_time(fileName.c_str(), access_time);
                    sys::path::replace_extension(fileName, KERNEL_FILE_EXTENSION);
                    fs::last_write_time(fileName.c_str(), access_time);
                    if (LLVM_UNLIKELY(codegen::TraceObjectCache)) {
                        errs() << "Read cache file: " << moduleId << KERNEL_FILE_EXTENSION << "\n";
                    }
                    return CacheObjectResult::CACHED;
                }
            } else if (LLVM_UNLIKELY(codegen::TraceObjectCache)) {
                errs() << "Failed to load cache file: " << moduleId << KERNEL_FILE_EXTENSION << "\n";
            }

        }

invalid:

        kernel->makeModule(b);
        Module * const module = kernel->getModule();
        // mark this module as cachable
        module->getOrInsertNamedMetadata(CACHEABLE);
        // if this module has a signature, add it to the metadata
        if (LLVM_UNLIKELY(kernel->hasSignature())) {
            NamedMDNode * const md = module->getOrInsertNamedMetadata(SIGNATURE);
            assert (md->getNumOperands() == 0);
            MDString * const sig = MDString::get(module->getContext(), signature);
            assert (!isNonMatchingSignature(sig, signature));
            md->addOperand(MDNode::get(module->getContext(), {sig}));
        }

    } else { // uncachable
        kernel->makeModule(b);
    }
    mKnownSignatures.emplace(signature, kernel->getModule());
    return CacheObjectResult::UNCACHED;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief notifyObjectCompiled
 *
 * A new module has been compiled. If it is cacheable and no conflicting module exists, write it out.
 ** ------------------------------------------------------------------------------------------------------------- */
void ParabixObjectCache::notifyObjectCompiled(const Module * M, MemoryBufferRef Obj) {

    if (LLVM_LIKELY(M->getNamedMetadata(CACHEABLE) != nullptr)) {

        const StringRef moduleId(M->getModuleIdentifier());

        Path objectName(mCachePath);
        sys::path::append(objectName, CACHE_PREFIX);
        objectName.append(moduleId);
        objectName.append(OBJECT_FILE_EXTENSION);

        // Write the object code
        std::error_code EC;
        raw_fd_ostream objFile(objectName, EC, sys::fs::F_None);
        if (LLVM_UNLIKELY(EC)) {
            SmallVector<char, 512> tmp;
            llvm::raw_svector_ostream msg(tmp);
            msg << "Could not write to \""
                << objectName.str()
                << "\" in object cache directory.\n\n"
                "Reason: " << EC.message() << "\n\n"
                "Rerun " << codegen::ProgramName << " with --enable-object-cache=0";
            report_fatal_error(msg.str());
        }
        objFile.write(Obj.getBufferStart(), Obj.getBufferSize());
        objFile.close();

        sys::path::replace_extension(objectName, KERNEL_FILE_EXTENSION);
        raw_fd_ostream kernelFile(objectName.str(), EC, sys::fs::F_None);

        if (LLVM_UNLIKELY(EC)) {
            SmallVector<char, 512> tmp;
            llvm::raw_svector_ostream msg(tmp);
            msg << "Could not write to \""
                << objectName.str()
                << "\" in object cache directory.\n\n"
                "Reason: " << EC.message() << "\n\n"
                "Rerun " << codegen::ProgramName << " with --enable-object-cache=0";
            report_fatal_error(msg.str());
        }

        // Clone the function prototypes and metadata to minimize the size of the stored .kernel file.
        std::unique_ptr<Module> H(new Module(moduleId, M->getContext()));
        for (const Function & f : M->getFunctionList()) {
            if (f.hasExternalLinkage() && !f.empty()) {
                Function::Create(f.getFunctionType(), Function::ExternalLinkage, f.getName(), H.get());
            }
        }
        const MDString * const sig = getSignature(M);
        if (sig) {
            NamedMDNode * const md = H->getOrInsertNamedMetadata(SIGNATURE);
            assert (md->getNumOperands() == 0);
            MDString * const sigCopy = MDString::get(H->getContext(), sig->getString());
            md->addOperand(MDNode::get(H->getContext(), {sigCopy}));
        }

        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(7, 0, 0)
        WriteBitcodeToFile(H.get(), kernelFile);
        #else
        WriteBitcodeToFile(*H, kernelFile);
        #endif
        kernelFile.close();

        if (LLVM_UNLIKELY(codegen::TraceObjectCache)) {
            errs() << "Wrote cache file: " << moduleId << KERNEL_FILE_EXTENSION << "\n";
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getObject
 ** ------------------------------------------------------------------------------------------------------------- */
std::unique_ptr<MemoryBuffer> ParabixObjectCache::getObject(const Module * module) {
    const auto moduleId = module->getModuleIdentifier();
    const auto f = mCachedObject.find(moduleId);
    if (f == mCachedObject.end()) {
        return nullptr;
    }
    llvm::MemoryBuffer * const buffer = f->second.release();
    #ifndef NDEBUG
    if (LLVM_UNLIKELY(buffer == nullptr)) {
        SmallVector<char, 512> tmp;
        llvm::raw_svector_ostream msg(tmp);
        msg << "getObject called multiple times for \""
            << moduleId
            << "\".\n\n";
        report_fatal_error(msg.str());
    }
    #else
    mCachedObject.erase(f);
    #endif
    return std::unique_ptr<llvm::MemoryBuffer>(buffer);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCacheCleanUp
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool ParabixObjectCache::requiresCacheCleanUp() noexcept {
    if (LLVM_UNLIKELY(mStartedCacheCleanupDaemon)) {
        return false;
    }
    // if we cannot lock the pid file then an earlier process
    // must have acquired it.
    return FileLock{fs::path{mCachePath.str()}}.locked();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiateCacheCleanUp
 ** ------------------------------------------------------------------------------------------------------------- */
void ParabixObjectCache::initiateCacheCleanUp() noexcept {
    if (LLVM_UNLIKELY(requiresCacheCleanUp())) {
        mStartedCacheCleanupDaemon = true;
        const auto pid = fork();
        if (pid == 0) {
            char * const cachePath = const_cast<char *>(mCachePath.c_str());
            char * args[3] = {const_cast<char *>(CACHE_JANITOR_FILE_NAME), cachePath, nullptr};
            Path janitorFileName(codegen::ProgramName);
            sys::path::remove_filename(janitorFileName);
            sys::path::append(janitorFileName, CACHE_JANITOR_FILE_NAME);
            char * const janitorPath = const_cast<char *>(janitorFileName.c_str());
            if (execvp(janitorPath, args) < 0) {
                #ifndef NDEBUG
                SmallVector<char, 1024> tmp;
                raw_svector_ostream out(tmp);
                out << "failed to exec cache cleanup deamon \"" << janitorPath << "\"";
                perror(reinterpret_cast<const char *>(out.str().bytes().begin()));
                #endif
                exit(errno);
            }
        } else if (pid < 0) {
            #ifndef NDEBUG
            perror("failed to fork cache cleanup deamon process");
            #endif
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

    const auto err = sys::fs::create_directories(mCachePath, true);

    if (LLVM_UNLIKELY(err)) {
        std::string tmp;
        llvm::raw_string_ostream msg(tmp);
        msg << "Could not create object cache directory \""
            << mCachePath.str() << "\" with read/write permissions.\n\n"
            "Reason: " << err.message() << "\n\n"
            "Rerun " << codegen::ProgramName << " with --enable-object-cache=0";
        report_fatal_error(msg.str());
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief saveCachePath
 ** ------------------------------------------------------------------------------------------------------------- */
inline void ParabixObjectCache::saveCacheSettings() noexcept {


}

ParabixObjectCache::ParabixObjectCache() {
    loadCacheSettings();
    initiateCacheCleanUp();
}

/** ------------------------------------------------------------------------------------------------------------- *
+* @brief destructor
+** ------------------------------------------------------------------------------------------------------------- */
ParabixObjectCache::~ParabixObjectCache() { }