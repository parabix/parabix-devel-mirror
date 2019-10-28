/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

// This file provides a forward delcation for all RE ADT types.
// Include re/adt/adt.h for full type definitions.

#pragma once

namespace re {

class RE;

class Name; class Start; class End;   class CC; 
class Seq;  class Alt;   class Rep;   class Intersect; 
class Diff; class Range; class Group; class Assertion;
class Capture; class Reference;

}
