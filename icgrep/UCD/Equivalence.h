#ifndef EQUIVALENCE_H
#define EQUIVALENCE_H
/*
 *  Copyright (c) 2018 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 */
#include <UCD/unicode_set.h>

namespace UCD {
    
enum EquivalenceOptions {Canonical = 0, Caseless = 1, Compatible = 2};
    
bool hasOption(enum EquivalenceOptions optionSet, enum EquivalenceOptions testOption);

UnicodeSet equivalentCodepoints(codepoint_t, EquivalenceOptions options = Canonical);
    
}

#endif
