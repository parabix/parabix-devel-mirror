# Parabix
> Parabix technology is a high-performance programming framework for streaming text processing applications, leveraging both SIMD and multicore parallel processing features.

[![pipeline status](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/badges/master/pipeline.svg)](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/-/commits/master) [![Gitter](https://badges.gitter.im/parabix-devel/community.svg)](https://gitter.im/parabix-devel/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)

### Documentation
For more information about Parabix, please check our [wiki](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/-/wikis/home) or reach out to us on [Gitter](https://gitter.im/parabix-devel/community).

### Tools
Parabix includes a set of built-in [tools](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/-/tree/master/tools), such as [`icgrep`](README-icgrep.md), which is a very fast regular expression search program.

### Requirements

To build Parabix, you need a development environment that meets a few requirements.

-  Standard C++ development tools including git, C++, etc.
-  A modern C++ compiler supporting at least C++ 11.
-  The [`cmake`](https://cmake.org/download/) build system version 2.8 or better.
-  [`Boost`](https://www.boost.org/users/download/) libraries version `1.61` or better.
-  An [`LLVM`](https://releases.llvm.org/download.html) system version `3.9` or better.

### Build

- Clone parabix-devel from the repository:
  `git clone https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel.git`
- Create a build subdirectory
  - `cd parabix-devel`
  - `mkdir build`
  - `cd build`
- Create the makefiles
  `cmake -DCMAKE_BUILD_TYPE=Release ..`
  - _Note: if you have built/installed a custom LLVM version, you may need to override cmake's default search path._
`cmake -DCMAKE_PREFIX_PATH=path/to/libllvm -DCMAKE_BUILD_TYPE=Release ..`
- `make`
- If you wish to run the test suite, while you are still in the build directory, run the following command:
`make check`

In the configuration above, all compiled tools will then be found on `/path/to/parabix-devel/build/bin`

### Papers

[Bitwise Data Parallelism with LLVM: The ICgrep Case Study](https://link.springer.com/chapter/10.1007%2F978-3-319-27122-4_26). Robert D. Cameron, Nigel Medforth, Dan Lin, Dale Denis, William N. Sumner. ICA3PP (2) 2015: 373-387

[Bitwise data parallelism in regular expression matching](https://dl.acm.org/doi/10.1145/2628071.2628079). Robert D. Cameron, Thomas C. Shermer, Arrvindh Shriraman, Kenneth S. Herdy, Dan Lin, Benjamin R. Hull, Meng Lin. PACT 2014: 139-150

[icXML: Accelerating a Commercial XML Parser Using SIMD and Multicore Technologies](http://www.balisage.net/Proceedings/vol10/html/Cameron01/BalisageVol10-Cameron01.html). Presented at Balisage: The Markup Conference 2013, Montréal, Canada, August 6 - 9, 2013.

[Parabix: Boosting the Efficiency of Text Processing on Commodity Processors](http://www.cs.sfu.ca/~ashriram/publications/2012_HPCA_Parabix.pdf), 18th International Symposium on High Performance Computer Architecture in New Orleans, Louisiana, February 2012

[Parallel Scanning with Bitstream Addition: An XML Case Study](http://parabix.costar.sfu.ca/export/901/docs/EuroPar2011/europar-cameron.pdf), Euro-Par 2011 Conference, Bordeaux, France, September 2011.

[Parallel Bit Stream Technology as a Foundation for XML Parsing Performance](http://www.balisage.net/Proceedings/vol4/html/Cameron01/BalisageVol4-Cameron01.html), 2009 International Symposium on Processing XML Efficiently: Overcoming Limits on Space, Time, or Bandwidth, Montreal, Canada, August 2009.

[Architectural Support for SIMD Text Processing with Parallel Bit Streams: The Inductive Doubling Principle](http://parabix.costar.sfu.ca/export/901/docs/ASPLOS09/asplos094-cameron.pdf), Fourteenth International Conference on Architectural Support for Programming Languages and Operating Systems (ASPLOS '09), Washington, DC, March 2009.

### License
LLVM files are governed by the LLVM Release License in [LLVM-LICENSE.txt](LLVM-LICENSE.txt). `Parabix` is governed by Open Software License `3.0` in [OSL-3.0.txt](OSL-3.0.txt).

