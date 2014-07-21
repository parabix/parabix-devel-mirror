README-icgrep-0.8.txt

This is the open-source version of icgrep 0.9.   

To build icgrep, you need an installed LLVM system providing the
core libraries.  The distribution includes a suitable source
code version of LLVM.

To build LLVM, 
(L1) open a terminal window and cd to the llvm-build directory
(L2) enter the following command to build the makefiles
cmake -DCMAKE_INSTALL_PREFIX=../libllvm -DLLVM_TARGETS_TO_BUILD=X86 -DLLVM_BUILD_TOOLS=OFF -DLLVM_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/clang++ -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/clang ../llvm-3.4.2.src
(L3) Still in the llvm-build directory, enter the commands "make" and then "make install"

Using the installed LLVM, building icgrep uses the CMake build
system generator.   
(IC1)  open a terminal window and cd to the icgrep-build directory
(IC2)  enter the following command to build the makefiles
cmake -DCMAKE_CXX_FLAGS="-O3 -D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS" -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/clang++ -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/clang ../icgrep
(IC3) Enter the command "make" 

LLVM files are governed by the LLVM Release License in LLVM-LICENSE.txt.
icgrep is governed by Open Software License 3.0 in OSL-3.0.txt.


