#!/usr/bin/env python3

import os
import subprocess
import argparse
import sys
import copy
import itertools
import logging
from functools import reduce
from datetime import datetime

def readFile(filename):
    with open(filename, 'r') as f:
        return [line[:-1] for line in f]

def saveInFile(filename, what):
    with open(filename, 'a') as f:
        f.write(what + "\n")

# apply functions in a pipe, e.g.: 3 |> str |> print would print "3"
def pipe(data, *funcs):
    for func in funcs:
        data = func(data)
    return data

def stripString(s, begin, end, startFrom=None):
    decoded = str(s)
    beginIdx = 0
    if startFrom is not None:
        beginIdx = decoded.find(startFrom)
    posB = decoded.find(begin, beginIdx) + len(begin)
    posE = decoded.find(end, posB)
    return decoded[posB:posE]

def stripPerfStatTime(s, padding="per insn"):
    spaces = " " * len(padding)
    out = stripString(s, "\\n\\n", " seconds", "of all branches" + spaces)
    return (float(out.strip()), out.strip())

def stripVersions(s):
    llvmVersion = stripString(s, "LLVM version ", "\\n")
    unicodeVersion = stripString(s, "Unicode version ", "\\n")
    parabixRevision = stripString(s, "Parabix revision ", "\\n")
    hostCPU = stripString(s, "Host CPU: ", "\\n")
    target = stripString(s, "Default target: ", "\\n")
    return [llvmVersion, unicodeVersion, parabixRevision, hostCPU, target]

def runAndReturnSizeFile(s, filename):
    out = str(os.path.getsize(filename))
    return [out]

def runProc(command, timeout):
    with subprocess.Popen(command, stderr=subprocess.PIPE) as popen:
        try:
            _, errs = popen.communicate(timeout=timeout)
            return errs
        except Exception as e:
            popen.kill()
            raise e

def crashIfNotAllNumbers(arr):
    for value in arr:
        try:
            float(value)
        except Exception as e:
            raise e

def run(what, filename, regex, delimiter=", ", timeout=30, otherFlags=[]):
    versionCmd = what + ["--version"]
    versionInfo = stripVersions(subprocess.check_output(versionCmd))
    command = what + otherFlags + ["-enable-object-cache=0"]
    runtimeStr = ""
    try:
        perfCmd = ["perf", "stat"] + command
        (time, out) = stripPerfStatTime(runProc(perfCmd, timeout=timeout))
        runtimeStr = str(out)
        logging.info("runtime out: " + runtimeStr)
    except Exception as e:
        runtimeStr = str(sys.maxsize)
        logging.error("error raised: ", e)
    return (runtimeStr, versionInfo)

def mkname(regex, target, buildfolder):
    buildpath = os.path.join(buildfolder, "bin/icgrep")
    command = [buildpath, regex, target]
    logging.info("root command: " + " ".join(command))
    return command

def save(runtime, testname, regex, versionInfo, outfile):
    now = datetime.now().strftime("%m/%d/%Y")
    val = ",".join([now, runtime, testname, regex] + versionInfo)
    saveInFile(outfile, val)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-f", "--file", dest="csvfile", default=os.path.join('.', 'file.csv'), help="output filepath")
    argparser.add_argument("-b", "--build-path", dest="buildfolder", default=os.path.join('.', 'build'), help="icgrep build folder")
    argparser.add_argument("-x", "--expression", dest="regex", default="[a-c]", help="Regular expression")
    argparser.add_argument("-t", "--target", dest="target", default=os.path.join('.', 'script.py'), help="File target for comparison")
    argparser.add_argument("-z", "--logfile", dest="logfile", default=os.path.join('.', 'log'), help="log file for debugging")
    argparser.add_argument("-n", "--testname", dest="testname", default="unnamed", help="The alias for the test")
    args, otherFlags = argparser.parse_known_args()

    logging.basicConfig(filename=args.logfile, filemode='w', level=logging.DEBUG)

    pipe(
        mkname(args.regex, args.target, args.buildfolder),
        lambda c: run(c, args.target, args.regex, otherFlags=otherFlags),
        lambda res: save(res[0], args.testname, args.regex, res[1] + otherFlags, args.csvfile)
    )

