#ifndef DEBUG_MESSAGES_HPP
#define DEBUG_MESSAGES_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

#ifdef PRINT_DEBUG_MESSAGES

#define MODE (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH)

void PipelineCompiler::debugInit(BuilderRef b) {
    mThreadId = b->CreatePThreadSelf();
//    SmallVector<char, 256> tmp;
//    raw_svector_ostream out(tmp);
//    out << codegen::ProgramName << ".debug.%" PRIx64;
//    const auto format = out.str();
//    mDebugFileName = b->CreateMalloc(b->getSize(format.size() + 16));
//    b->CreateSprintfCall(mDebugFileName, format, mThreadId);
//    Value * const fd = b->CreateOpenCall(mDebugFileName, b->getInt32(O_WRONLY | O_APPEND | O_CREAT | O_EXCL), b->getInt32(MODE));
//    mDebugFdPtr = b->CreateAllocaAtEntryPoint(fd->getType());
//    b->CreateStore(fd, mDebugFdPtr);
}


template <typename ... Args>
BOOST_NOINLINE void PipelineCompiler::debugPrint(BuilderRef b, StringRef format, Args ...args) const {
    SmallVector<char, 512> tmp;
    raw_svector_ostream out(tmp);
    out << "%016" PRIx64 << "  " << format << "\n";
    SmallVector<Value *, 8> argVals(3);
    argVals[0] = b->getInt32(STDERR_FILENO);
    argVals[1] = b->GetString(out.str());
    argVals[2] = mThreadId ? mThreadId : b->getSize(0);
    std::initializer_list<llvm::Value *> a{std::forward<Args>(args)...};
    argVals.append(a);
    b->CreateCall(b->GetDprintf(), argVals);
}

void PipelineCompiler::debugHalt(BuilderRef b) const {
//    Value * const fd = b->CreateLoad(mDebugFdPtr);
//    b->CreateCloseCall(fd);
}

void PipelineCompiler::debugResume(BuilderRef b) const {
//    Value * const fd = b->CreateOpenCall(mDebugFileName, b->getInt32(O_WRONLY | O_APPEND), b->getInt32(MODE));
//    b->CreateStore(fd, mDebugFdPtr);
}

void PipelineCompiler::debugClose(BuilderRef b) {
//    Value * const fd = b->CreateLoad(mDebugFdPtr);
//    b->CreateCloseCall(fd);
//    b->CreateStore(Constant::getNullValue(mDebugFdPtr->getType()->getPointerElementType()), mDebugFdPtr);
//    b->CreateFree(mDebugFileName);
}

#endif

}

#endif // DEBUG_MESSAGES_HPP
