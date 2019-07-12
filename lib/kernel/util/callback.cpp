/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <kernel/util/callback.h>
#include <llvm/Support/ErrorHandling.h>
using namespace kernel;


extern "C" void signal_dispatcher(intptr_t callback_object_addr, unsigned signal) {
    reinterpret_cast<SignallingObject *>(callback_object_addr)->handle_signal(signal);
}

void SignallingObject::handle_signal(unsigned s) {
    mSignalCount++;
    mLastSignal = s;
}

unsigned SignallingObject::getSignalCount() {
    return mSignalCount;
}

unsigned SignallingObject::getLastSignal() {
    return mLastSignal;
}

