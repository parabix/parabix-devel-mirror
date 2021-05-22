#if !defined(PAPI_INSTRUMENTATION_LOGIC_HPP) && defined(ENABLE_PAPI)
#define PAPI_INSTRUMENTATION_LOGIC_HPP

#ifdef ENABLE_PAPI
#include <papi.h>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include <codegen/TypeBuilder.h>
#endif

// TODO: merge cycle counter with papi?

#include "pipeline_compiler.hpp"

namespace kernel {

using papi_counter_t = unsigned long_long;

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

        tokenizer<escaped_list_separator<char>> events(codegen::PapiCounterOptions);
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
            out << "\n"
                   "Check papi_avail for available options.";
            report_fatal_error(out.str());
        }

        PAPI_destroy_eventset(&EventSet);
    }
    PAPI_shutdown();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPAPIEventCounterPipelineProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPAPIEventCounterPipelineProperties(BuilderRef b) {

    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        IntegerType * const papiCounterTy = TypeBuilder<papi_counter_t, false>::get(b->getContext());
        const auto numOfEvents = PAPIEventList.size();
        ArrayType * const papiEventCounterTotalTy = ArrayType::get(papiCounterTy, numOfEvents);

        // TODO: make a better method than this for accumulating the final thread local counts.
        // We can't share a global scalar that isn't guarded by synchronization but could pass
        // the state of thread 0 to all other threads during other users.

        // NOTE: it might actually be beneficial to guard the exit point for the pipeline but
        // this should be tested in the context of nested pipelines before committing to such
        // a design. The following choice to use a global accumulator in the thread local
        // destructor can easily be converted to doing this.

        mTarget->addInternalScalar(papiEventCounterTotalTy, STATISTICS_GLOBAL_PAPI_COUNT_ARRAY, 0);
        if (mNumOfThreads > 1) {
            mTarget->addThreadLocalScalar(papiEventCounterTotalTy, STATISTICS_THREAD_LOCAL_PAPI_COUNT_ARRAY, 0);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPAPIEventCounterKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPAPIEventCounterKernelProperties(BuilderRef b, const unsigned kernel, const bool /* isRoot */) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        IntegerType * const papiCounterTy = TypeBuilder<papi_counter_t, false>::get(b->getContext());
        const auto numOfEvents = PAPIEventList.size();
        ArrayType * const papiEventListCountersTy = ArrayType::get(papiCounterTy, numOfEvents);
        ArrayType * const papiDataTy = ArrayType::get(papiEventListCountersTy, NUM_OF_PAPI_COUNTERS);
        const auto prefix = makeKernelName(kernel) + STATISTICS_PAPI_COUNT_ARRAY_SUFFIX;
        mTarget->addInternalScalar(papiDataTy, prefix, kernel);
        /*
        if (isRoot) {
            mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_PARTITION_JUMP_SYNCHRONIZATION), kernel);
        }
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_BUFFER_EXPANSION), kernel);
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_BUFFER_COPY), kernel);
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_KERNEL_EXECUTION), kernel);
        mTarget->addInternalScalar(papiEventListCountersTy, prefix + std::to_string(PAPI_KERNEL_TOTAL), kernel);*/
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
            report_fatal_error("Internal linking error: unable to find PAPI_library_init");
        }
        ConstantInt * const version = ConstantInt::get(intTy, PAPI_VER_CURRENT);
        checkPAPIRetValAndExitOnError(b,  "PAPI_library_init", PAPI_VER_CURRENT, b->CreateCall(PAPIlibInitFn, { version }));
        if (mNumOfThreads > 1) {
            Function * PAPIThreadInitFn = m->getFunction("PAPI_thread_init");
            Function * pthreadSelfFn = m->getFunction("pthread_self");
            checkPAPIRetValAndExitOnError(b,  "PAPI_thread_init", PAPI_OK, b->CreateCall(PAPIThreadInitFn, pthreadSelfFn));
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

        Function * const PAPICreateEventSetFn = m->getFunction("PAPI_create_eventset");

        FixedArray<Value *, 1> createEventSetArgs;
        createEventSetArgs[0] = PAPIEventSet;
        Value * const createEventSetRetVal = b->CreateCall(PAPICreateEventSetFn, createEventSetArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_create_eventset", PAPI_OK, createEventSetRetVal);

        const auto n = PAPIEventList.size();
        Value * const eventSetCodeArray = b->CreateAllocaAtEntryPoint(intTy, b->getInt32(n));

        for (unsigned i = 0; i < n; ++i) {
            ConstantInt * const eventCode = ConstantInt::get(intTy, PAPIEventList[i]);
            b->CreateStore(eventCode, b->CreateGEP(eventSetCodeArray, b->getInt32(i)));
        }

        PAPIEventSetVal = b->CreateLoad(PAPIEventSet);
        assert (PAPIEventSetVal->getType() == intTy);


        FixedArray<Value *, 3> addEventArgs;
        addEventArgs[0] = PAPIEventSetVal;
        addEventArgs[1] = eventSetCodeArray;
        addEventArgs[2] = ConstantInt::get(intTy, n);

        Function * const PAPIAddEventsFn = m->getFunction("PAPI_add_events");

        Value * const addEventsRetVal = b->CreateCall(PAPIAddEventsFn, addEventArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_add_events", PAPI_OK, addEventsRetVal);

        Function * const PAPIStartFn = m->getFunction("PAPI_start");

        FixedArray<Value *, 1> startArgs;
        startArgs[0] = PAPIEventSetVal;

        Value * const startRetVal = b->CreateCall(PAPIStartFn, startArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_start", PAPI_OK, startRetVal);

        // PAPI_start starts counting all of the hardware events contained in the previously defined EventSet.
        // All counters are implicitly set to zero before counting.

        IntegerType * const papiCounterTy = TypeBuilder<papi_counter_t, false>::get(b->getContext());
        PointerType * const papiCounterPtrTy = papiCounterTy->getPointerTo();
        ArrayType * const papiEventListCountersTy = ArrayType::get(papiCounterTy, n);

        auto makeCounterList = [&]() {
            return b->CreatePointerCast(b->CreateAllocaAtEntryPoint(papiEventListCountersTy), papiCounterPtrTy);
        };

        PAPIReadInitialMeasurementArray = makeCounterList();
        PAPIReadBeforeMeasurementArray = makeCounterList();
        PAPIReadAfterMeasurementArray = makeCounterList();

    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readPAPIEventsIntoArray
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readPAPIMeasurement(BuilderRef b, Value * const measurementArray) const {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Module * const m = b->getModule();
        Function * const PAPIReadFn = m->getFunction("PAPI_read"); assert (PAPIReadFn);
        FixedArray<Value *, 2> args;
        args[0] = PAPIEventSetVal;
        args[1] = measurementArray;
        // TODO: should probably check the error code here but if we do get an error,
        // what can we avoid contaminating the results but also inform the user something
        // went wrong?
        b->CreateCall(PAPIReadFn, args);
    }
}




/** ------------------------------------------------------------------------------------------------------------- *
 * @brief accumPAPIMeasurementWithoutReset
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::accumPAPIMeasurementWithoutReset(BuilderRef b, Value * const beforeMeasurement, PAPIKernelCounter measurementType) const {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {

        readPAPIMeasurement(b, PAPIReadAfterMeasurementArray);

        const auto prefix = makeKernelName(mKernelId) + STATISTICS_PAPI_COUNT_ARRAY_SUFFIX;
        Value * const baseCounter = b->getScalarFieldPtr(prefix);

        ConstantInt * const ZERO = b->getInt32(0);

        FixedArray<Value *, 3> update;
        update[0] = ZERO;
        update[1] = b->getInt32((unsigned)measurementType);

        // TODO: this could be vectorized but would require storing the counters as vectortypes.
        // The cost of moving the values from general to SIMD registers might become a factor.
        // Does LLVM consider this when adding vectortypes?
        const auto n = PAPIEventList.size();
        for (unsigned i = 0; i < n; ++i) {
            Value * const offset = b->getInt32(i);
            Value * const beforeVal = b->CreateLoad(b->CreateGEP(beforeMeasurement, offset), "beforeVal");
            Value * const afterVal = b->CreateLoad(b->CreateGEP(PAPIReadAfterMeasurementArray, offset), "afterVal");
            if (LLVM_UNLIKELY(CheckAssertions)) {
                b->CreateAssert(b->CreateICmpULE(beforeVal, afterVal), "non-monotonic papi counter?");
            }
            Value * const diff = b->CreateSub(afterVal, beforeVal);
            update[2] = offset;
            Value * const ptr = b->CreateGEP(baseCounter, update);
            Value * const updatedVal = b->CreateAdd(b->CreateLoad(ptr), diff);

//            b->CallPrintInt("uPtr." + std::to_string(mKernelId) + "." + std::to_string(measurementType) + "." + std::to_string(i), ptr);
//            b->CallPrintInt("uVal." + std::to_string(mKernelId) + "." + std::to_string(measurementType) + "." + std::to_string(i), updatedVal);

            b->CreateStore(updatedVal, ptr);

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

        Value * finalEventReads = nullptr;
        if (mNumOfThreads > 1) {
            finalEventReads = b->getScalarFieldPtr(STATISTICS_THREAD_LOCAL_PAPI_COUNT_ARRAY);
        } else {
            finalEventReads = b->getScalarFieldPtr(STATISTICS_GLOBAL_PAPI_COUNT_ARRAY);
        }

        PointerType * const counterPtrTy = TypeBuilder<papi_counter_t, false>::get(b->getContext())->getPointerTo();

        finalEventReads = b->CreatePointerCast(finalEventReads, counterPtrTy);

        Function * const PAPIStopFn = m->getFunction("PAPI_stop");

        FixedArray<Value *, 2> stopArgs;
        stopArgs[0] = PAPIEventSetVal;
        stopArgs[1] = finalEventReads;
        Value * const stopRetVal = b->CreateCall(PAPIStopFn, stopArgs);
        checkPAPIRetValAndExitOnError(b,  "PAPI_stop", PAPI_OK, stopRetVal);

        FixedArray<Value *, 1> args;
        args[0] = PAPIEventSetVal;

        Function * const PAPICleanupEventsetFn = m->getFunction("PAPI_cleanup_eventset");

        Value * const cleanupRetVal = b->CreateCall(PAPICleanupEventsetFn, args);
        checkPAPIRetValAndExitOnError(b,  "PAPI_cleanup_eventset", PAPI_OK, cleanupRetVal);

        args[0] = PAPIEventSet;
        Function * const PAPIDestroyEventsetFn = m->getFunction("PAPI_destroy_eventset");
        Value * const destroyRetVal = b->CreateCall(PAPIDestroyEventsetFn, args);
        checkPAPIRetValAndExitOnError(b,  "PAPI_destroy_eventset", PAPI_OK, destroyRetVal);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief accumulateFinalPAPICounters
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::accumulateFinalPAPICounters(BuilderRef b) {
    if (LLVM_UNLIKELY(EnablePAPICounters && mNumOfThreads > 1)) {

        PointerType * const counterPtrTy = TypeBuilder<papi_counter_t, false>::get(b->getContext())->getPointerTo();

        Value * const localFinalEventReads = b->CreatePointerCast(b->getScalarFieldPtr(STATISTICS_THREAD_LOCAL_PAPI_COUNT_ARRAY), counterPtrTy);
        Value * const globalFinalEventReads = b->CreatePointerCast(b->getScalarFieldPtr(STATISTICS_GLOBAL_PAPI_COUNT_ARRAY), counterPtrTy);
        const auto n = PAPIEventList.size();
        for (unsigned i = 0; i < n; ++i) {
            Value * const offset = b->getInt32(i);
            Value * const val = b->CreateLoad(b->CreateGEP(localFinalEventReads, offset));
            Value * const ptr = b->CreateGEP(globalFinalEventReads, offset);
            Value * const updated = b->CreateAdd(val, b->CreateLoad(ptr));
            b->CreateStore(updated, ptr);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief shutdownPAPI
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::shutdownPAPI(BuilderRef b) const {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        Module * const m = b->getModule();
        Function * const PAPIShutdownFn = m->getFunction("PAPI_shutdown");
        b->CreateCall(PAPIShutdownFn);
    }
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkPAPIRetVal
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkPAPIRetValAndExitOnError(BuilderRef b, StringRef source, const int expected, Value * const retVal) const {

    assert (EnablePAPICounters);

    IntegerType * const intTy = TypeBuilder<int, false>::get(b->getContext());
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
        FunctionType * PAPI_strerrFnTy = FunctionType::get(int8PtrTy, {intTy}, false);
        PAPI_strerrFn = Function::Create(PAPI_strerrFnTy, Function::ExternalLinkage, "PAPI_strerror", m);
    }
    FixedArray<Value *, 1> strerrArgs;
    strerrArgs[0] = retVal;
    Value * const strerr = b->CreateCall(PAPI_strerrFn, strerrArgs);


    FixedArray<Value *, 4> args;
    args[0] = b->getInt32(STDERR_FILENO);
    args[1] = b->GetString("Error: %s returned %s\n");
    args[2] = b->GetString(source);
    args[3] = strerr;
    b->CreateCall(b->GetDprintf(), args);
    b->CreateExit(-1);
    b->CreateBr(onSuccess);

    b->SetInsertPoint(onSuccess);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief __print_pipeline_PAPI_report
 ** ------------------------------------------------------------------------------------------------------------- */
namespace {
extern "C"
BOOST_NOINLINE
void __print_pipeline_PAPI_report(const unsigned numOfKernels, const char ** kernelNames,
                                  const unsigned numOfEvents, const int * const eventCode,
                                  const papi_counter_t * const values,
                                  const papi_counter_t * const totals) {

                         // totals contains the final event counts for the program;
                         // values has numOfKernels * numOfEvents * numOfMeasurements
                         // event counts in that order.

    auto & out = errs();

    size_t maxNameLength = 4;
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto len = std::strlen(kernelNames[i]);
        maxNameLength = std::max(maxNameLength, len);
    }

    size_t maxEventLength = 8;
    char eventName[PAPI_MAX_STR_LEN + 1];
    for (unsigned i = 0; i != numOfEvents; i++) {
        const auto rval = PAPI_event_code_to_name(eventCode[i], eventName);
        if (LLVM_LIKELY(rval == PAPI_OK)) {
            maxEventLength = std::max(maxEventLength, std::strlen(eventName));
        }
    }

    papi_counter_t maxCounter = 5;
    for (unsigned i = 0; i != numOfEvents; i++) {
        maxCounter = std::max(maxCounter, totals[i]);
    }
    size_t maxCounterLength = 1;
    BEGIN_SCOPED_REGION
    papi_counter_t n = 10;
    while (maxCounter >= n) {
        n *= static_cast<papi_counter_t>(10);
        ++maxCounterLength;
    }
    maxCounterLength = std::max(maxCounterLength, 7UL);
    END_SCOPED_REGION

    out << "PAPI REPORT\n\n"
           "  # "  // kernel #
           "NAME"; // kernel Name
    assert (maxNameLength > 4);
    out.indent(maxNameLength - 3);

    out << "EVENT";
    assert (maxEventLength > 5);
    out.indent(maxEventLength - 4);

    out << "|   SYNC " // kernel synchronization %,
           "  PART " // partition synchronization %,
           "  EXPD " // buffer expansion %,
           "  COPY " // look ahead + copy back + look behind %,
           "  PIPE " // pipeline overhead %,
           "  EXEC " // execution %,
           "|"; // total kernel value.
    out.indent(maxCounterLength - 4);

    out << "VALUE       %\n";

//    PAPI_KERNEL_SYNCHRONIZATION
//    , PAPI_PARTITION_JUMP_SYNCHRONIZATION
//    , PAPI_BUFFER_EXPANSION
//    , PAPI_BUFFER_COPY

//    , PAPI_KERNEL_EXECUTION
//    , PAPI_KERNEL_TOTAL


    std::string kernelNameFmt;
    BEGIN_SCOPED_REGION
    raw_string_ostream formatter(kernelNameFmt);
    formatter << "%3" PRIu32 " " // kernel #
            "%-" << maxNameLength << "s" // kernel name
            " ";
    END_SCOPED_REGION

    std::string eventNameFmt;
    BEGIN_SCOPED_REGION
    raw_string_ostream formatter(eventNameFmt);
    formatter << "%-" << maxEventLength << "s" // event name
                 " |";
    END_SCOPED_REGION


    std::string valueFmt;
    BEGIN_SCOPED_REGION
    raw_string_ostream formatter(valueFmt);
    formatter << " | %" << maxCounterLength << PRIuMAX "  %6.2f\n";
    END_SCOPED_REGION

    // data is in [kernels][counters][events] ordering

    #define GET_POS(kernel, event, counter) \
        ((kernel * NUM_OF_PAPI_COUNTERS + (counter)) * numOfEvents + event)

    // std::vector<papi_counter_t> measurementSubtotals(numOfEvents * NUM_OF_PAPI_COUNTERS, 0UL);

    for (unsigned i = 0; i < numOfKernels; ++i) {
        for (unsigned j = 0; j < numOfEvents; ++j) {
            if (j == 0) {
                out << llvm::format(kernelNameFmt.data(), i + 1, kernelNames[i]);
            } else {
                out.indent(maxNameLength + 5);
            }

            const auto rval = PAPI_event_code_to_name(eventCode[j], eventName);
            if (LLVM_LIKELY(rval == PAPI_OK)) {
                out << llvm::format(eventNameFmt.data(), (char*)eventName);
            } else {
                out.write_hex(eventCode[j]);
            }

            const auto subtotal = values[GET_POS(i, j, PAPI_KERNEL_TOTAL)];
            const long double fsubtotal = subtotal;

            for (unsigned k = 0; k < PAPI_KERNEL_EXECUTION; ++k) {
                const long double val = values[GET_POS(i, j, k)];
                assert (val < subtotal);
                const auto r = (val / fsubtotal);
                assert (0.0L <= r && r <= 1.0L);
                out << llvm::format(" %6.2f", ((double)(r)) * 100.0);
            }

            BEGIN_SCOPED_REGION
            papi_counter_t sum = 0;
            for (unsigned k = 0; k < PAPI_KERNEL_TOTAL; ++k) {
                sum += values[GET_POS(i, j, k)];
            }
            const papi_counter_t other = (sum < subtotal) ? subtotal - sum : 0.0;
            const long double fother = other;
            const auto r = (fother / fsubtotal);
            assert (0.0L <= r && r <= 1.0L);
            out << llvm::format(" %6.2f", (double)(r) * 100.0);
            END_SCOPED_REGION

            BEGIN_SCOPED_REGION
            const long double val = values[GET_POS(i, j, PAPI_KERNEL_EXECUTION)];
            const auto r = (val / fsubtotal);
            assert (0.0L <= r && r <= 1.0L);
            out << llvm::format(" %6.2f", (double)(r) * 100.0);
            END_SCOPED_REGION

            BEGIN_SCOPED_REGION
            const long double ftotal = totals[j];
            const auto r = (fsubtotal / ftotal);
            assert (0.0L <= r && r <= 1.0L);
            const double ratio = (double)(r) * 100.0;
            out << llvm::format(valueFmt.data(), subtotal, ratio);
            END_SCOPED_REGION

        }
    }

    #undef GET_POS

    out << '\n';

}

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printPAPIReport
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printPAPIReportIfRequested(BuilderRef b) {
    if (LLVM_UNLIKELY(EnablePAPICounters)) {
        std::vector<Constant *> kernelNames;
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            kernelNames.push_back(b->GetString(kernel->getName()));
        }

        ConstantInt * const ZERO = b->getInt32(0);

        const auto numOfKernels = LastKernel - FirstKernel + 1U;

        auto toGlobal = [&](ArrayRef<Constant *> array, Type * const type, size_t size) {
            ArrayType * const arTy = ArrayType::get(type, size);
            Constant * const ar = ConstantArray::get(arTy, array);
            Module & mod = *b->getModule();            
            GlobalVariable * const gv = new GlobalVariable(mod, arTy, true, GlobalValue::PrivateLinkage, ar);
            FixedArray<Value *, 2> tmp;
            tmp[0] = ZERO;
            tmp[1] = ZERO;
            return b->CreateInBoundsGEP(gv, tmp);
        };

        PointerType * const int8PtrTy = b->getInt8PtrTy();

        Value * const arrayOfKernelNames = toGlobal(kernelNames, int8PtrTy, numOfKernels);

        PointerType * const counterPtrTy = TypeBuilder<papi_counter_t, false>::get(b->getContext())->getPointerTo();

        IntegerType * const intTy = TypeBuilder<int, false>::get(b->getContext());

        const auto numOfEvents = PAPIEventList.size();
        SmallVector<Constant *, 8> eventCodes;
        for (unsigned i = 0; i < numOfEvents; ++i) {
            eventCodes.push_back(ConstantInt::get(intTy, PAPIEventList[i]));
        }
        Value * const arrayOfEventCodes = toGlobal(eventCodes, intTy, numOfEvents);

        const auto BYTES_PER_KERNEL_COUNTER_GROUP = numOfEvents * sizeof(papi_counter_t) * NUM_OF_PAPI_COUNTERS;
        Constant * const requiredSpace =  b->getSize(numOfKernels * BYTES_PER_KERNEL_COUNTER_GROUP);

        Value * const values = b->CreatePointerCast(b->CreateAlignedMalloc(requiredSpace, sizeof(papi_counter_t)), int8PtrTy);
        Constant * const bytesPerKernelGroup = b->getSize(BYTES_PER_KERNEL_COUNTER_GROUP);

        FixedArray<Value *, 3> indices;
        indices[0] = ZERO;
        for (size_t i = FirstKernel, offset = 0; i <= LastKernel; ++i) {
            Value * const baseCounter = b->getScalarFieldPtr(makeKernelName(i) + STATISTICS_PAPI_COUNT_ARRAY_SUFFIX);
            Constant * const kernelOutputOffset = b->getSize(offset);
            Value * const ptr = b->CreateGEP(values, kernelOutputOffset);
            b->CreateMemCpy(ptr, baseCounter, bytesPerKernelGroup, sizeof(papi_counter_t));
            offset += BYTES_PER_KERNEL_COUNTER_GROUP;
        }

        Value * const totals = b->getScalarFieldPtr(STATISTICS_GLOBAL_PAPI_COUNT_ARRAY);

        FixedArray<Value *, 6> args;
        args[0] = ConstantInt::get(intTy, numOfKernels);
        args[1] = arrayOfKernelNames;
        args[2] = ConstantInt::get(intTy, numOfEvents);
        args[3] = arrayOfEventCodes;
        args[4] = b->CreatePointerCast(values, counterPtrTy);
        args[5] = b->CreatePointerCast(totals, counterPtrTy);

        Function * const reportPrinter = b->getModule()->getFunction("__print_pipeline_PAPI_report");
        assert (reportPrinter);
        b->CreateCall(reportPrinter, args);

        b->CreateFree(values);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief linkPAPILibrary
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::linkPAPILibrary(BuilderRef b) {
    b->LinkFunction("PAPI_library_init", PAPI_library_init);
    b->LinkFunction("PAPI_thread_init", PAPI_thread_init);
    b->LinkFunction("PAPI_create_eventset", PAPI_create_eventset);
    b->LinkFunction("PAPI_add_events", PAPI_add_events);
    b->LinkFunction("PAPI_start", PAPI_start);
    b->LinkFunction("PAPI_read", PAPI_read);
    b->LinkFunction("PAPI_stop", PAPI_stop);
    b->LinkFunction("PAPI_cleanup_eventset", PAPI_cleanup_eventset);
    b->LinkFunction("PAPI_destroy_eventset", PAPI_destroy_eventset);
    b->LinkFunction("PAPI_shutdown", PAPI_shutdown);
    b->LinkFunction("PAPI_strerror", PAPI_strerror);

    b->LinkFunction("__print_pipeline_PAPI_report", __print_pipeline_PAPI_report);
}


}

#endif // PAPI_INSTRUMENTATION_LOGIC_HPP
