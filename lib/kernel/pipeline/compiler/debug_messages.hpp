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
        mThreadId = b->CreatePThreadSelf();
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
    SmallVector<char, 512> tmp;
    raw_svector_ostream out(tmp);
    SmallVector<Value *, 8> argVals(2);
    argVals[0] = b->getInt32(STDERR_FILENO);
    if (mThreadId) {
        out << "%016" PRIx64 "  ";
        argVals.push_back(mThreadId);
    }
    out << format << "\n";
    argVals[1] = b->GetString(out.str());
    argVals.append(std::initializer_list<llvm::Value *>{std::forward<Args>(args)...});
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
