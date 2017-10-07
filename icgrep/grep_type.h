/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ICGREP_GREP_TYPE_H
#define ICGREP_GREP_TYPE_H

enum class GrepType {
    Normal
    , NameExpression
    , PropertyValue
    , CallBack
};

enum class GrepSource {
    File
    , StdIn
    , Internal
};

#endif //ICGREP_GREP_TYPE_H
