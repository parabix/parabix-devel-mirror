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

def createCSV(filename, delimiter=', ', opt=1):
    if not os.path.exists(filename):
        if opt == 1:
            value  = "datetime, filename, regular expression, LLVM version, Unicode version, parabix revision, "
            value += "host CPU, target triple, full command, opt level, compile time, total time, asm size"
        else:
            value = "runtime, command"
        with open(filename, 'a') as f:
            f.write(value + "\n")

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

def mkname(folder, regex, target, flags, buildfolder):
    buildpath = os.path.join(buildfolder, os.path.join(folder, "bin/icgrep"))
    command = [buildpath, regex, target] + flags
    logging.info("root command: " + " ".join(command))
    return command

def breakFlagsIfNeeded(flags):
    newFlags = []
    for f in flags:
        newFlags.extend(f.split())
    return newFlags

def findLLVMFolders(llvmsfile):
    if not os.path.exists(llvmsfile):
        return [""]
    else:
        return readFile(llvmsfile)

def save(res, outfile, runFile):
    (runtime, allOutputs) = res
    for output in allOutputs:
        saveInFile(outfile, output)
    best = 0
    for i in range(1, len(runtime)):
        (besttime, _) = runtime[best]
        (comptime, _) = runtime[i]
        if comptime < besttime:
            best = i
    (time, command) = runtime[best]
    val = str(time) + ", " + " ".join(command)
    saveInFile(runFile, val)
    return pipe(map(lambda t: t[0], runtime), list)

def reduceFlags(flags, idx):
    return (idx-1, flags[:idx-1] + flags[idx:])

def didFinish(idx):
    return idx <= 0

def betterOrEqualRuntime(lhs, rhs, bias=0.15):
    fCount = lambda acc, x: 0 if not x else acc + 1
    res = []
    for i in range(0, len(rhs)):
        compare = list(map(lambda x, y: x < (y + bias), lhs[i], rhs[i]))
        res.append(reduce(fCount, compare, 0))
    mostLLVMBetter = reduce(fCount, res, 0) >= (len(rhs) / 2)
    return mostLLVMBetter

if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-c", "--flags-file", dest="flags", default=os.path.join('.', 'flags'), help="flags filepath")
    argparser.add_argument("-f", "--final-file", dest="finalfile", default=os.path.join('.', 'output.csv'), help="output filepath")
    argparser.add_argument("-r", "--runtime-file", dest="runtimefile", default=os.path.join('.', 'runtime.csv'), help="runtime filepath")
    argparser.add_argument("-l", "--llvm-file", dest="llvms", default=os.path.join('.', 'llvms'), help="LLVM filepath")
    argparser.add_argument("-b", "--build-path", dest="buildfolder", default=os.path.join('.', 'build'), help="LLVM build folder")
    argparser.add_argument("-x", "--expression", dest="regex", default="[a-c]", help="Regular expression")
    argparser.add_argument("-t", "--target", dest="target", default=os.path.join('.', 'script.py'), help="File target for comparison")
    argparser.add_argument("-z", "--logfile", dest="logfile", default=os.path.join('.', 'log'), help="log file for debugging")
    argparser.add_argument("-o", "--optlevels", dest="optlevels", default=["none", "less"], help="opt levels for test")
    args, otherFlags = argparser.parse_known_args()

    logging.basicConfig(filename=args.logfile, filemode='w', level=logging.DEBUG)

    createCSV(args.finalfile, opt=1)
    createCSV(args.runtimefile, opt=2)
    flagset = readFile(args.flags)
    folders = findLLVMFolders(args.llvms)
    initFlags = [[], flagset, None]
    nfolders = 1 if not folders else len(folders) 
    bestFlagsRuntime = [[sys.maxsize] * len(args.optlevels)] * nfolders
    bestFlags = flagset
    for flags in initFlags:
        converged = False
        idx = len(bestFlags)
        while not converged:
            (idx, impFlags) = reduceFlags(bestFlags, idx) if flags is None else (idx, flags)
            mapFn = lambda folder: pipe(
                        impFlags,
                        breakFlagsIfNeeded,
                        lambda flgs: mkname(folder, args.regex, args.target, flgs, args.buildfolder),
                        lambda c: run(c, args.target, args.regex, optLevels=args.optlevels, otherFlags=otherFlags),
                        lambda res: save(res, args.finalfile, args.runtimefile)
                    )
            impFlagsRuntime = pipe(map(mapFn, folders), list)
            if (betterOrEqualRuntime(impFlagsRuntime, bestFlagsRuntime)):
                bestFlagsRuntime = impFlagsRuntime
                bestFlags = impFlags
            converged = flags != None or didFinish(idx)
        logging.info("Best flags: " + str(bestFlags))
