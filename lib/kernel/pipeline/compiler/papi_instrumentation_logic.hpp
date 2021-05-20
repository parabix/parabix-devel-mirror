#if !defined(PAPI_INSTRUMENTATION_LOGIC_HPP) && defined(ENABLE_PAPI)
#define PAPI_INSTRUMENTATION_LOGIC_HPP

#ifdef ENABLE_PAPI
#include <papi.h>
#include <boost/tokenizer.hpp>
#include <codegen/TypeBuilder.h>
#endif

// TODO: merge cycle counter with papi?

#include "pipeline_compiler.hpp"

namespace kernel {

using papi_counter_t = long_long;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief convertPAPIEventNamesToCodes
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::convertPAPIEventNamesToCodes() {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {

        const int rvalInit = PAPI_library_init(PAPI_VER_CURRENT);
        if (rvalInit != PAPI_VER_CURRENT) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "PAPI Library Init Error: ";
            out << PAPI_strerror(rvalInit);
            report_fatal_error(out.str());
        }

        tokenizer<escaped_list_separator<char> > events(codegen::PapiCounterOptions);

        for (const auto & event : events) {
            int EventCode = PAPI_NULL;
            const int rvalEventNameToCode = PAPI_event_name_to_code(event.c_str(), &EventCode);
            if (LLVM_LIKELY(rvalEventNameToCode == PAPI_OK)) {
                PAPIEventList.push_back(EventCode);
            } else {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                out << "PAPI Library cannot resolve event name: ";
                out << event.c_str();
                out << "\n";
                out << PAPI_strerror(rvalEventNameToCode);
                report_fatal_error(out.str());
            }
        }

        // sanity test whether this event set is valid
        int EventSet = PAPI_NULL;
        const auto rvalCreateEventSet = PAPI_create_eventset(&EventSet);
        if (rvalCreateEventSet != PAPI_OK) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "PAPI Create Event Set Error: ";
            out << PAPI_strerror(rvalCreateEventSet);
            report_fatal_error(out.str());
        }

        const auto rvalAddEvents = PAPI_add_events(EventSet, PAPIEventList.data(), (int)PAPIEventList.size());

        if (rvalAddEvents != PAPI_OK) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "PAPI Add Events Error: ";
            out << PAPI_strerror(rvalCreateEventSet < PAPI_OK ? rvalCreateEventSet : PAPI_EINVAL);
            report_fatal_error(out.str());
        }

        PAPI_destroy_eventset(&EventSet);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPAPIEventCounterPipelineProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPAPIEventCounterPipelineProperties(BuilderRef b) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        IntegerType * const papiCounterTy = TypeBuilder<papi_counter_t, false>::get(b->getContext());
        const auto n = PAPIEventList.size();
        ArrayType * const papiEventListCountersTy = ArrayType::get(papiCounterTy, n);
        mTarget->addThreadLocalScalar(papiEventListCountersTy, STATISTICS_GLOBAL_PAPI_COUNT_ARRAY, 0);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPAPIEventCounterKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPAPIEventCounterKernelProperties(BuilderRef b, const unsigned kernel) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        IntegerType * const papiCounterTy = TypeBuilder<papi_counter_t, false>::get(b->getContext());
        const auto n = PAPIEventList.size();
        ArrayType * const papiEventListCountersTy = ArrayType::get(papiCounterTy, n);
        const auto prefix = makeKernelName(kernel) + STATISTICS_PAPI_COUNT_ARRAY_SUFFIX;

        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_KERNEL_SYNCHRONIZATION), kernel);
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_BUFFER_MANAGEMENT), kernel);
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_KERNEL_EXECUTION), kernel);
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_KERNEL_TOTAL), kernel);
    }
}




/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePAPIAndCreateEventSet
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializePAPI(BuilderRef b) const {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Module * const m = b->getModule();
        Function * PAPIlibInitFn = m->getFunction("PAPI_library_init");
        IntegerType * const intTy = TypeBuilder<int, false>::get(b->getContext());
        if (LLVM_LIKELY(PAPIlibInitFn == nullptr)) {
            FunctionType * PAPIlibInitFnTy = FunctionType::get(intTy, {intTy}, false);
            PAPIlibInitFn = Function::Create(PAPIlibInitFnTy, Function::ExternalLinkage, "PAPI_library_init", m);
        }
        ConstantInt * const version = ConstantInt::get(intTy, PAPI_VER_CURRENT);
        checkPAPIRetValAndExitOnError(b,  "PAPI_library_init", PAPI_VER_CURRENT, b->CreateCall(PAPIlibInitFn, { version }));
        if (mNumOfThreads > 1) {
            Value * const pthreadSelf = b->CreatePThreadSelf();
            Function * PAPIThreadInitFn = m->getFunction("PAPI_thread_init");
            if (LLVM_LIKELY(PAPIThreadInitFn == nullptr)) {
                FunctionType * PAPIThreadInitFnTy = FunctionType::get(intTy, {pthreadSelf->getType()}, false);
                PAPIThreadInitFn = Function::Create(PAPIThreadInitFnTy, Function::ExternalLinkage, "PAPI_thread_init", m);
            }
            checkPAPIRetValAndExitOnError(b,  "PAPI_thread_init", PAPI_OK, b->CreateCall(PAPIThreadInitFn, { pthreadSelf }));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief registerPAPIThread
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::registerPAPIThread(BuilderRef /* b */) const {
    // PAPI documentation indicates this and unregister thread are not necessary.
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePAPIAndCreateEventSet
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::createEventSetAndStartPAPI(BuilderRef b) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Module * const m = b->getModule();

        IntegerType * const intTy = TypeBuilder<int, false>::get(b->getContext());
        PAPIEventSet = b->CreateAllocaAtEntryPoint(intTy);
        b->CreateStore(ConstantInt::get(intTy, PAPI_NULL), PAPIEventSet);

        Function * PAPICreateEventSetFn = m->getFunction("PAPI_create_eventset");
        if (PAPICreateEventSetFn == nullptr) {
            FunctionType * PAPICreateEventSetFnTy = FunctionType::get(intTy, {PAPIEventSet->getType()}, false);
            PAPICreateEventSetFn = Function::Create(PAPICreateEventSetFnTy, Function::ExternalLinkage, "PAPI_create_eventset", m);
        }
        FixedArray<Value *, 1> createEventSetArgs;
        createEventSetArgs[0] = PAPIEventSet;
        Value * const createEventSetRetVal = b->CreateCall(PAPICreateEventSetFn, createEventSetArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_create_eventset", PAPI_OK, createEventSetRetVal);

        FixedArray<Value *, 2> offsets;
        offsets[0] = b->getInt32(0);

        const auto n = PAPIEventList.size();
        Value * const eventSetCodeArray = b->CreateAllocaAtEntryPoint(intTy, b->getInt32(n));
        for (unsigned i = 0; i < n; ++i) {
            ConstantInt * const eventCode = ConstantInt::get(intTy, PAPIEventList[i]);
            offsets[1] = b->getInt32(i);
            b->CreateStore(eventCode, b->CreateGEP(eventSetCodeArray, offsets));
        }

        PAPIEventSetVal = b->CreateLoad(PAPIEventSet);

        FixedArray<Value *, 3> addEventArgs;
        addEventArgs[0] = PAPIEventSetVal;
        addEventArgs[1] = eventSetCodeArray;
        addEventArgs[2] = ConstantInt::get(intTy, n);

        Function * PAPIAddEventsFn = m->getFunction("PAPI_add_events");
        if (PAPIAddEventsFn == nullptr) {
            FunctionType * PAPIAddEventsFnTy = FunctionType::get(intTy, {intTy, eventSetCodeArray->getType(), intTy}, false);
            PAPIAddEventsFn = Function::Create(PAPIAddEventsFnTy, Function::ExternalLinkage, "PAPI_add_events", m);
        }
        Value * const addEventsRetVal = b->CreateCall(PAPIAddEventsFn, addEventArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_add_events", PAPI_OK, addEventsRetVal);

        Function * PAPIStartFn = m->getFunction("PAPI_start");
        if (PAPIAddEventsFn == nullptr) {
            FunctionType * PAPIStartFnTy = FunctionType::get(intTy, {intTy}, false);
            PAPIStartFn = Function::Create(PAPIStartFnTy, Function::ExternalLinkage, "PAPI_start", m);
        }
        Value * const startRetVal = b->CreateCall(PAPIStartFn, addEventArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_start", PAPI_OK, startRetVal);

        // PAPI_start starts counting all of the hardware events contained in the previously defined EventSet.
        // All counters are implicitly set to zero before counting.

        IntegerType * const papiCounterTy = TypeBuilder<papi_counter_t, false>::get(b->getContext());
        ArrayType * const papiEventListCountersTy = ArrayType::get(papiCounterTy, n);

        PAPIReadMeasurementArray = b->CreateAllocaAtEntryPoint(papiEventListCountersTy, b->getInt32(PAPI_MEASUREMENTS_PER_KERNEL));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readPAPIEventsIntoArray
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readPAPIMeasurement(BuilderRef b, PAPIMeasurement counter) {
    Value * measurementArray = nullptr;
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Module * const m = b->getModule();

        IntegerType * const intTy = TypeBuilder<int, false>::get(b->getContext());

        FixedArray<Value *, 2> offset;
        offset[0] = b->getInt32(0);
        offset[1] = b->getInt32((unsigned)counter);
        measurementArray = b->CreateGEP(PAPIReadMeasurementArray, offset);

        Function * PAPIReadFn = m->getFunction("PAPI_read");
        if (PAPIReadFn == nullptr) {
            FunctionType * PAPIReadFnTy = FunctionType::get(intTy, {intTy, measurementArray->getType()}, false);
            PAPIReadFn = Function::Create(PAPIReadFnTy, Function::ExternalLinkage, "PAPI_read", m);
        }

        FixedArray<Value *, 2> args;
        args[0] = PAPIEventSetVal;
        args[1] = measurementArray;
        Value * const readRetVal = b->CreateCall(PAPIReadFn, args);
    }
    return measurementArray;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordPAPIKernelMeasurementDiff
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordPAPIKernelMeasurement(BuilderRef b, PAPIMeasurement from, PAPIKernelCounter sumTo) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Value * const after = readPAPIMeasurement(b, PAPI_KERNEL_AFTER);

        FixedArray<Value *, 2> offset;
        offset[0] = b->getInt32(0);
        offset[1] = b->getInt32((unsigned)from);
        Value * const before = b->CreateGEP(PAPIReadMeasurementArray, offset);

        const auto prefix = makeKernelName(mKernelId) + STATISTICS_PAPI_COUNT_ARRAY_SUFFIX;
        Value * const counter = b->getScalarFieldPtr(prefix + std::to_string(sumTo));

         // TODO: this could be vectorized but would require storing the counters as vectortypes.

        const auto n = PAPIEventList.size();
        for (unsigned i = 0; i < n; ++i) {
            offset[1] = b->getInt32(i);
            Value * const beforeVal = b->CreateLoad(b->CreateGEP(before, offset));
            Value * const afterVal = b->CreateLoad(b->CreateGEP(after, offset));
            Value * const diff = b->CreateSub(beforeVal, afterVal);
            Value * const ptr = b->CreateGEP(counter, offset);
            b->CreateStore(b->CreateAdd(b->CreateLoad(ptr), diff), ptr);

        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief unregisterPAPIThread
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::unregisterPAPIThread(BuilderRef /* b */) const {
    // PAPI documentation indicates this and register thread are not necessary.
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief stopPAPIAndDestroyEventSet
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::stopPAPIAndDestroyEventSet(BuilderRef b) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {

        Module * const m = b->getModule();
        IntegerType * const intTy = TypeBuilder<int, false>::get(b->getContext());

        Value * const finalEventReads = b->getScalarFieldPtr(STATISTICS_GLOBAL_PAPI_COUNT_ARRAY);

        Function * PAPIStopFn = m->getFunction("PAPI_stop");
        if (PAPIStopFn == nullptr) {
            FunctionType * PAPIStopFnTy = FunctionType::get(intTy, {intTy, finalEventReads->getType()}, false);
            PAPIStopFn = Function::Create(PAPIStopFnTy, Function::ExternalLinkage, "PAPI_stop", m);
        }

        FixedArray<Value *, 2> stopArgs;
        stopArgs[0] = PAPIEventSetVal;
        stopArgs[1] = finalEventReads;
        Value * const stopRetVal = b->CreateCall(PAPIStopFn, stopArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_stop", PAPI_OK, stopRetVal);

        FixedArray<Value *, 1> args;
        args[0] = PAPIEventSet;

        Function * PAPICleanupEventsetFn = m->getFunction("PAPI_cleanup_eventset");
        if (PAPICleanupEventsetFn == nullptr) {
            FunctionType * PAPICleanupEventsetFnTy = FunctionType::get(intTy, {PAPIEventSet->getType()}, false);
            PAPICleanupEventsetFn = Function::Create(PAPICleanupEventsetFnTy, Function::ExternalLinkage, "PAPI_cleanup_eventset", m);
        }
        Value * const cleanupRetVal = b->CreateCall(PAPICleanupEventsetFn, args);
        checkPAPIRetValAndExitOnError(b,  "PAPI_cleanup_eventset", PAPI_OK, cleanupRetVal);

        Function * PAPIDestroyEventsetFn = m->getFunction("PAPI_destroy_eventset");
        if (PAPICleanupEventsetFn == nullptr) {
            FunctionType * PAPIDestroyEventsetFnTy = FunctionType::get(intTy, {PAPIEventSet->getType()}, false);
            PAPIDestroyEventsetFn = Function::Create(PAPIDestroyEventsetFnTy, Function::ExternalLinkage, "PAPI_destroy_eventset", m);
        }
        Value * const destroyRetVal = b->CreateCall(PAPIDestroyEventsetFn, args);
        checkPAPIRetValAndExitOnError(b,  "PAPI_destroy_eventset", PAPI_OK, destroyRetVal);

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief shutdownPAPI
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::shutdownPAPI(BuilderRef b) const {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Module * const m = b->getModule();
        Function * PAPIShutdownFn = m->getFunction("PAPI_shutdown");
        if (LLVM_LIKELY(PAPIShutdownFn == nullptr)) {
            FunctionType * PAPIShutdownFnTy = FunctionType::get(b->getVoidTy(), {}, false);
            PAPIShutdownFn = Function::Create(PAPIShutdownFnTy, Function::ExternalLinkage, "PAPI_shutdown", m);
        }
        b->CreateCall(PAPIShutdownFn);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkPAPIRetVal
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkPAPIRetValAndExitOnError(BuilderRef b, StringRef source, const int expected, Value * const retVal) const {
    assert (EnablePAPICounters);
    IntegerType * const intTy = cast<IntegerType>(retVal->getType());
    ConstantInt * const papiOk = ConstantInt::get(intTy, expected);

    BasicBlock * const current = b->GetInsertBlock();
    Function * const function = current->getParent();
    Module * const m = function->getParent();
    BasicBlock * onError = BasicBlock::Create(b->getContext(), source + "_onError", function, current->getNextNode());
    BasicBlock * onSuccess = BasicBlock::Create(b->getContext(), source + "_onSuccess", function, onError);

    Value * const isOk = b->CreateICmpEQ(retVal, papiOk);
    b->CreateLikelyCondBr(isOk, onSuccess, onError);

    b->SetInsertPoint(onError);
    Function * PAPI_strerrFn = m->getFunction("PAPI_strerror");
    if (PAPI_strerrFn == nullptr) {
        PointerType * const int8PtrTy = b->getInt8PtrTy();
        FunctionType * PAPI_strerrFnTy = FunctionType::get(intTy, {intTy, int8PtrTy, intTy}, false);
        PAPI_strerrFn = Function::Create(PAPI_strerrFnTy, Function::ExternalLinkage, "PAPI_strerror", m);
    }
    FixedArray<Value *, 4> args;
    args[0] = b->getInt32(STDERR_FILENO);
    args[1] = b->GetString("Error: %s returned %s\n");
    args[2] = b->GetString(source);
    args[3] = b->CreateCall(PAPI_strerrFn, {});
    b->CreateCall(b->GetDprintf(), args);
    b->CreateExit(-1);
    b->CreateBr(onSuccess);

    b->SetInsertPoint(onSuccess);
}

}

#endif // PAPI_INSTRUMENTATION_LOGIC_HPP
