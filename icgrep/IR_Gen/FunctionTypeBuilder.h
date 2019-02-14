#ifndef FUNCTIONTYPEBUILDER_H
#define FUNCTIONTYPEBUILDER_H

#include <IR_Gen/TypeBuilder.h>

// NOTE: Currently, LLVM TypeBuilder can deduce FuntionTypes for up to 5 arguments. The following
// templates have no limit but should be deprecated if the TypeBuilder ever supports n-ary functions.

namespace {

template<unsigned i, typename... Args>
struct ParameterTypeBuilder;

template<unsigned i, typename A1, typename... An>
struct ParameterTypeBuilder<i, A1, An...> {
    static void get(llvm::LLVMContext & C, llvm::Type ** params) {
        ParameterTypeBuilder<i, A1>::get(C, params);
        ParameterTypeBuilder<i + 1, An...>::get(C, params);
    }
};

template<unsigned i, typename A>
struct ParameterTypeBuilder<i, A> {
    static void get(llvm::LLVMContext & C, llvm::Type ** params) {
        params[i] = llvm::TypeBuilder<A, false>::get(C);
    }
};

}

template<typename T>
struct FunctionTypeBuilder;

template<typename R, typename... Args>
struct FunctionTypeBuilder<R(Args...)> {
    static llvm::FunctionType * get(llvm::LLVMContext & C) {
        llvm::Type * params[sizeof...(Args)];
        ParameterTypeBuilder<0, Args...>::get(C, params);
        return llvm::FunctionType::get(llvm::TypeBuilder<R, false>::get(C), params, false);
    }
};

template<typename R>
struct FunctionTypeBuilder<R()> {
    static llvm::FunctionType * get(llvm::LLVMContext & C) {
        return llvm::FunctionType::get(llvm::TypeBuilder<R, false>::get(C), false);
    }
};

#endif // FUNCTIONTYPEBUILDER_H
