/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

namespace pablo {

String * CodeGenState::getString(const std::string string) {
    auto f = mStringMap.find(string);
    String * result;
    if (f == mStringMap.end()) {
        result = makeString(string);
        mStringMap.insert(std::make_pair(*result, result));
    }
    else {
        result = f->second;
    }
    return result;
}

Call * CodeGenState::createCall(const std::string name) {
    return makeCall(getString(name));
}

Var * CodeGenState::createVar(const std::string name) {
    return makeVar(getString(name));
}

Var * CodeGenState::createVar(const Assign * assign) {
    return makeVar(assign);
}

CharClass * CodeGenState::createCharClass(const std::string name) {
    return makeCharClass(getString(name));
}

Assign *CodeGenState::createAssign(const std::string name, PabloE * expr) {
    return makeAssign(getString(name), expr);
}

}
