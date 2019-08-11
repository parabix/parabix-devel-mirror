#!/usr/bin/env python3

from subprocess import *
import sys

def invoke(exe):
    print("testing %s..." % (exe))
    retcode = call([exe])
    if retcode != 0:
        print("\u001b[31mfailed\u001b[0m\n")
    else:
        print("\u001b[32mpassed\u001b[0m\n")
    return retcode

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("USAGE: python3 run_kernel_tests.py <test-exe ...>")
        exit(1)
    print("\n--- Kernel Test Suite: Running %d Tests ---\n" % (len(sys.argv) - 1))
    failcount = 0
    for i in range(1, len(sys.argv)):
        rt = invoke(sys.argv[i])
        if rt != 0:
            failcount += 1
    if failcount == 0:
        print("\u001b[32mpassed\u001b[0m all kernel tests")
    else:
        print("\u001b[31mfailed\u001b[0m %d of %d kernel tests" % (failcount, len(sys.argv) - 1))
