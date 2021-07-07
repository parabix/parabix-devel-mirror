#ifndef GREP_TOOLCHAIN_H
#define GREP_TOOLCHAIN_H

namespace grep {

extern int Threads;
extern bool PabloTransposition;
extern bool SplitTransposition;
extern bool UnicodeIndexing;
extern bool PropertyKernels;
extern bool MultithreadedSimpleRE;
extern int ScanMatchBlocks;
extern int MatchCoordinateBlocks;
extern unsigned ByteCClimit;
extern bool TraceFiles;

}

#endif // GREP_TOOLCHAIN_H
