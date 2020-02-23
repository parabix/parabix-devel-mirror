README-icgrep.txt

This is the open-source version of icgrep 1.x.     

icgrep is a very fast regular expression search program, particularly
for complex regular expressions.  It is also a very capable engine,
supporting most common regular expression syntax and many useful
command line options.   

icgrep 1.0 is designed to offer substantial Unicode support, meeting
all the Unicode Level 1 requirements of UTS #18, the Unicode
Technical Standard for regular expressions.  Development of icgrep 2.0 is
on track to meet the Unicode level 2 requirements of UTS #18.

Normal usage to find lines in a file f matching a regexp r is:
icgrep r f

To produce a count of matching lines only, use the command:
icgrep -c r f

To read the regexp to be matched from file regexpf use the command:

icgrep -f regexpf f


BUILD

To build icgrep, you need a development environment that meets
several requirements.
-  A modern C++ compiler supporting C++ 11.
-  The cmake build system version 2.8 or better.
-  Boost libraries version 1.61 or better.
-  Standard C++ development tools including git, C++, etc.
-  An LLVM system version 3.9 or better.

Clone parabix-devel from the repository:
git clone https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel.git

Create a build subdirectory
cd parabix-devel
mkdir build

Create the makefiles
cmake -DCMAKE_BUILD_TYPE=Release

Note: if you have built/installed a custom LLVM version, you may need
to override cmake's default search path.

cmake -DCMAKE_PREFIX_PATH=path/to/libllvm -DCMAKE_BUILD_TYPE=Release ..

LLVM files are governed by the LLVM Release License in LLVM-LICENSE.txt.
icgrep is governed by Open Software License 3.0 in OSL-3.0.txt.
