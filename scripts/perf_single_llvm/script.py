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

def stripVersions(s):
    llvmVersion = stripString(s, "LLVM version ", "\\n")
    unicodeVersion = stripString(s, "Unicode version ", "\\n")
    parabixRevision = stripString(s, "Parabix revision ", "\\n")
    hostCPU = stripString(s, "Host CPU: ", "\\n")
    target = stripString(s, "Default target: ", "\\n")
    return [llvmVersion, unicodeVersion, parabixRevision, hostCPU, target]

def stripIcGrepCompileTime(s):
    out = stripString(s, "Execution Time: ", " seconds", "Kernel Generation\\n")
    return [out.strip()]

def stripPerfStatTime(s, padding="per insn"):
    spaces = " " * len(padding)
    out = stripString(s, "\\n\\n", " seconds", "of all branches" + spaces)
    return (float(out.strip()), [out.strip()])

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

# Append to the CSV file in the format
#
# datetime, filename, regular expression, LLVM version, Unicode version, parabix revision,
# host CPU, target triple, full command, opt level, compile time, total time, asm size
def run(what, filename, regex, delimiter=", ", timeout=30, asmFile="asm", optLevels=[], otherFlags=[]):
    baseOutput = [str(datetime.now()), filename, regex]
    versionCmd = what + ["--version"]
    baseOutput += stripVersions(subprocess.check_output(versionCmd))
    logging.info("version command: " + " ".join(versionCmd))
    command = what + otherFlags + ["-enable-object-cache=0"]
    baseOutput += [" ".join(command)]
    runtime = []
    allOutputs = [copy.deepcopy(baseOutput) for _ in range(len(optLevels))]
    for (idx, optLevel) in enumerate(optLevels):
        try:
            allOutputs[idx] += [optLevel]
            commandOptLevel = command + ["-backend-optimization-level=" + optLevel]
            timeKernelCmd = commandOptLevel + ["-time-kernels"]
            icgrepCompTime = stripIcGrepCompileTime(runProc(timeKernelCmd, timeout=timeout))
            crashIfNotAllNumbers(icgrepCompTime)
            allOutputs[idx] += icgrepCompTime
            logging.info("time kernel command: " + " ".join(timeKernelCmd))
            perfCmd = ["perf", "stat"] + commandOptLevel
            (time, out) = stripPerfStatTime(runProc(perfCmd, timeout=timeout))
            allOutputs[idx] += out
            runtime += [(time, commandOptLevel)]
            logging.info("perf stat command: " + " ".join(perfCmd))
            asmCmd = commandOptLevel + ["-ShowASM=" + asmFile]
            allOutputs[idx] += runAndReturnSizeFile(runProc(asmCmd, timeout=timeout), asmFile)
            logging.info("asm command: " + " ".join(perfCmd))
        except Exception as e:
            # 3 represents the values for compile time, total time and asm size
            allOutputs[idx] += ["inf"] * 3
            runtime += [(sys.maxsize, command)]
            logging.error("error raised: ", e)
            continue
    finalOutputs = []
    for output in allOutputs:
        finalOutputs += [delimiter.join(output)]
    return (runtime, finalOutputs)

def mkname(regex, target, buildfolder):
    buildpath = os.path.join(buildfolder, "bin/icgrep")
    command = [buildpath, regex, target]
    logging.info("root command: " + " ".join(command))
    return command

def save(res, outfile):
    now = datetime.now().strftime("%m/%d/%Y")
    val = now + ", " + " ".join(res)
    saveInFile(outfile, val)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-f", "--file", dest="csvfile", default=os.path.join('.', 'file.csv'), help="output filepath")
    argparser.add_argument("-b", "--build-path", dest="buildfolder", default=os.path.join('.', 'build'), help="icgrep build folder")
    argparser.add_argument("-x", "--expression", dest="regex", default="[a-c]", help="Regular expression")
    argparser.add_argument("-t", "--target", dest="target", default=os.path.join('.', 'script.py'), help="File target for comparison")
    argparser.add_argument("-z", "--logfile", dest="logfile", default=os.path.join('.', 'log'), help="log file for debugging")
    args, otherFlags = argparser.parse_known_args()

    logging.basicConfig(filename=args.logfile, filemode='w', level=logging.DEBUG)

    pipe(
        mkname(args.regex, args.target, args.buildfolder),
        lambda c: run(c, args.target, args.regex, otherFlags=otherFlags),
        lambda res: save(res, args.finalfile)
    )

