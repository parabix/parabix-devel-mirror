README-icgrep-0.8.txt

This is the open-source version of icgrep 0.8.   This file includes
an executable for 64-bit Linux systems (compiled specifically for
Ubuntu 12.04) as well as instructions for building in other contexts.
The executable is in icgrep-0.8/icgrep-build/icgrep

To build icgrep, you need an installed LLVM system providing the
core libraries.    One is included with this distributed in the 
libllvm directory.    

Using the installed LLVM, building icgrep uses the CMake build
system generator.   
(IC1)  open a terminal window and cd to the icgrep-build directory
(IC2)  enter the following command to build the makefiles
cmake -DCMAKE_CXX_FLAGS="-O3 -D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS" -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/clang++ -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/clang ../icgrep
(IC3) Enter the command "make" 

To rebuild LLVM, 
(L1) download a source distribution from llvm.org
and place inside the icgrep-0.8 directory, e.g., llvm-3.4.1.src
(L2) open a terminal window and cd to the llvm-build directory
(L3)  enter the following command to build the makefiles
cmake -DCMAKE_INSTALL_PREFIX=../libllvm -DLLVM_TARGETS_TO_BUILD=X86 -DLLVM_BUILD_TOOLS=OFF -DLLVM_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/clang++ -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/clang ../llvm-3.4.1.src
(L4) Still in the llvm-build directory, enter the commands "make" and then "make install"

Now complete icgrep installation using steps IC1 to IC3 above.

LLVM files are governed by the LLVM Release License in LLVM-LICENSE.txt.
icgrep is governed by Open Software License 3.0 in OSL-3.0.txt.
.

