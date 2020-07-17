# icgrep
> This is the open-source version of `icgrep 1.x`

icgrep is a very fast regular expression search program, particularly for complex regular expressions. It is also a very capable engine, supporting most common regular expression syntax and many useful command line options.

### Usage
`icgrep 1.0` is designed to offer substantial Unicode support, meeting **all** the Unicode Level 1 requirements of UTS #18, the Unicode Technical Standard for regular expressions.  Development of `icgrep 2.0` is on track to meet the Unicode level 2 requirements of UTS #18.

##### Regex matching

Normal usage to find lines in a file `f` matching a regular expression `r` is:
`icgrep r f`

##### Counting matching lines
To produce a count of matching lines only, use the flag `-c` such as below:
`icgrep -c r f`


To read the regular expression to be matched from file `regexpf` use the flag `-f` such as below:
`icgrep -f regexpf f`

### Build

`icgrep` is one of the tools available on `Parabix`. Check the [README.md](README.md) file for more information.

### License

LLVM files are governed by the LLVM Release License in [LLVM-LICENSE.txt](LLVM-LICENSE.txt). `icgrep` is governed by Open Software License 3.0 in [OSL-3.0.txt](OSL-3.0.txt).

