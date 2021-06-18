#ifndef DEBUG_MESSAGES_HPP
#define DEBUG_MESSAGES_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

#ifdef PRINT_DEBUG_MESSAGES

#define NEW_FILE (O_WRONLY | O_APPEND | O_CREAT | O_EXCL)

#define APPEND_FILE (O_WRONLY | O_APPEND)

#define MODE (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH)

void PipelineCompiler::debugInit(BuilderRef b) {
    if (codegen::SegmentThreads > 1) {
        Function * const pthreadSelfFn = b->getModule()->getFunction("pthread_self");
        mThreadId = b->CreateCall(pthreadSelfFn);
    } else {
        mThreadId = nullptr;
    }
//    SmallVector<char, 256> tmp;
//    raw_svector_ostream out(tmp);
//    out << codegen::ProgramName << ".debug.%" PRIx64;
//    const auto format = out.str();
//    mDebugFileName = b->CreateMalloc(b->getSize(format.size() + 16));
//    b->CreateSprintfCall(mDebugFileName, format, mThreadId);
//    Value * const fd = b->CreateOpenCall(mDebugFileName, b->getInt32(NEW_FILE), b->getInt32(MODE));
//    mDebugFdPtr = b->CreateAllocaAtEntryPoint(fd->getType());
//    b->CreateStore(fd, mDebugFdPtr);
}


template <typename ... Args>
BOOST_NOINLINE void PipelineCompiler::debugPrint(BuilderRef b, Twine format, Args ...args) const {
    #ifdef PRINT_DEBUG_MESSAGES_FOR_KERNEL_NUM
    const std::initializer_list<unsigned> L{0,PRINT_DEBUG_MESSAGES_FOR_KERNEL_NUM};
    if (std::find(L.begin(), L.end(), mKernelId) == L.end()) {
        return;
    }
    #endif
    SmallVector<char, 512> tmp;
    raw_svector_ostream out(tmp);
    #ifdef PRINT_DEBUG_MESSAGES_INCLUDE_THREAD_NUM
    if (mThreadId) {
        out << "%016" PRIx64 "  ";
    }
    #endif
    out << format << "\n";

    SmallVector<Value *, 8> argVals(2);
    argVals[0] = b->getInt32(STDERR_FILENO);
    argVals[1] = b->GetString(out.str());
    #ifdef PRINT_DEBUG_MESSAGES_INCLUDE_THREAD_NUM
    if (mThreadId) {
        argVals.push_back(mThreadId);
    }
    #endif
    argVals.append(std::initializer_list<llvm::Value *>{std::forward<Args>(args)...});
    #ifndef NDEBUG
    for (Value * arg : argVals) {
        assert ("null argument given to debugPrint" && arg);
    }
    #endif
    b->CreateCall(b->GetDprintf(), argVals);
}

void PipelineCompiler::debugHalt(BuilderRef b) const {
//    Value * const fd = b->CreateLoad(mDebugFdPtr);
//    b->CreateCloseCall(fd);
}

void PipelineCompiler::debugResume(BuilderRef b) const {
//    Value * const fd = b->CreateOpenCall(mDebugFileName, b->getInt32(APPEND_FILE), b->getInt32(MODE));
//    b->CreateStore(fd, mDebugFdPtr);
}

void PipelineCompiler::debugClose(BuilderRef b) {
//    Value * const fd = b->CreateLoad(mDebugFdPtr);
//    b->CreateCloseCall(fd);
//    b->CreateStore(Constant::getNullValue(mDebugFdPtr->getType()->getPointerElementType()), mDebugFdPtr);
//    b->CreateFree(mDebugFileName);
}

#undef NEW_FILE
#undef APPEND_FILE
#undef MODE

#endif

}

#endif // DEBUG_MESSAGES_HPP
