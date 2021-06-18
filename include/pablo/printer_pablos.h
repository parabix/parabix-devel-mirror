/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

namespace llvm { class raw_ostream; }

namespace pablo {

class PabloKernel;
class PabloBlock;
class Statement;
class PabloAST;

class PabloPrinter {
public:
    static void print(PabloAST const * node, llvm::raw_ostream & out) noexcept;
    static void print(PabloKernel const * kernel, llvm::raw_ostream & out) noexcept;
    static void print(PabloBlock const * block, llvm::raw_ostream & out, const bool expandNested = false, unsigned indent = 0) noexcept;
    static void print(Statement const * stmt, llvm::raw_ostream & out, const bool expandNested = false, unsigned indent = 0) noexcept;    
};


}
