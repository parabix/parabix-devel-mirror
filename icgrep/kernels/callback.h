/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef CALLBACK_H
#define CALLBACK_H

/* This file defines the callback interface from Parabix kernels
   to C++ routines for the purpose of sending signals.  */

#include <stdint.h>

namespace kernel {
    
/*  To implement a callback, the following steps are required.
 
    1.  A suitable handle_signal method is required.   The default
        handle_signal method simply counts the number of signals
        received and the value of the last signal received.
        If different behaviour is needed, define a subclass of SignallingObject
        with a suitable overridden handle_signal method.
 
    2.  Pass the address of your callback object as an input scalar
        to the kernel.
 
    3.  Create a LLVM Call to the signal dispatcher.  Assume the address
        of the call back object is stored in "handler_address".  The
        following code is illustrative.
        Value * handler = b->getScalarField("handler_address");
        Function * const dispatcher = m->getFunction("signal_dispatcher");
        b->CreateCall(dispatcher, {handler, theSignal});
 
    4.  Ensure that the address of the signal_dispatcher is available
        to the JIT engine (should be automatic with LoadDynamicLibraryPermanently).
        driver->LinkFunction(theKernel, "signal_dispatcher", &signal_dispatcher);
 
*/
 
class SignallingObject {
public:
    SignallingObject() : mSignalCount(0), mLastSignal(0) {}
    virtual ~SignallingObject() {}
    virtual void handle_signal(unsigned signal);
    unsigned getSignalCount();
    unsigned getLastSignal();
private:
    unsigned mSignalCount;
    unsigned mLastSignal;
};

extern "C" void signal_dispatcher(intptr_t callback_object_addr, unsigned signal);

}

#endif
