README-icgrep-0.9.txt

This is the open-source version of icgrep 0.9.   icgrep is a
very fast regular expression search program, particularly
for complex regular expressions.

Normal usage to find lines in a file f matching a regexp r is:
icgrep r f

To produce a count of matching lines only, use the command:
icgrep -c r f

To read the regexp to be matched from file rf use the command:

icgrep -f rf f

icgrep supports standard egrep syntax, except for Posix character
classes.   icgrep supports searches using ASCII or UTF-8.
Unicode two letter classes are supported, e.g. \p{Ll} matching
lower case Unicode letters and \P{Lu} matching all characters
that are not upper case letters.

Full Unicode level 1 support is planned for icgrep 1.0.

BUILD

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
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/clang++ -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/clang ../icgrep
(IC3) Enter the command "make"

LLVM files are governed by the LLVM Release License in LLVM-LICENSE.txt.
icgrep is governed by Open Software License 3.0 in OSL-3.0.txt.

### Experimental 256-bit BLOCK_SIZE
(IC1)  open a terminal window and cd to the icgrep-build directory
(IC2)  enter the following command to build the makefiles
cmake -DBLOCK_SIZE_256=on -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/g++ -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc ../icgrep
(IC3) Enter the command "make"


