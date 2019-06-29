#
# greptest.py - Functional correctness testing for grep implementations.
# Robert D. Cameron, Dec. 28, 2013
# Licensed under Academic Free License 3.0
#
# Uses an XML test suite with the following format.
# <greptest>
# <datafile id="simple1">
# A few lines of input
# in this simple test file
# provide fodder for some simple
# regexp tests.
# </datafile>
#
# <grepcase regexp="in" datafile="simple1" greplines="1 2"/>
# <grepcase regexp="[A-Z]" datafile="simple1" greplines="1"/>
#
# </greptest>


import sys, subprocess, os, optparse, re, codecs, stat
import xml.parsers.expat
import sys
import codecs
import random

sys.stdout = codecs.getwriter('utf8')(sys.stdout)
sys.stderr = codecs.getwriter('utf8')(sys.stderr)

in_datafile = False
dataFileName = ""

fileContents = {}

def getFileContents(fileName):
    if not fileName in fileContents:
        outfpath = os.path.join(options.datafile_dir, fileName)
        f = codecs.open(outfpath, encoding='utf-8', mode='r')
        fileContents[fileName] = f.read()
        f.close()
    return fileContents[fileName]

def start_element_open_file(name, attrs):
    global outf
    global outfpath
    global in_datafile
    global dataFileName
    if name == 'datafile':
        idFound = False
        for a in attrs:
            if a == 'id':
                dataFileName = attrs[a]
                idFound = True
        if not idFound:
            print("Expecting id attribute for datafile, but none found.")
            exit(-1)
        outfpath = os.path.join(options.datafile_dir, dataFileName)
        if options.utf16: outf = codecs.open(outfpath, encoding='utf-16BE', mode='w')
        else: outf = codecs.open(outfpath, encoding='utf-8', mode='w')
        in_datafile = True

def char_data_write_contents(data):
    global in_datafile
    if in_datafile:
        outf.write(data)

def max_count(grepCount, ):
    if "-l" in flags:
        if len(grepLines) > 0:
            return fileName
        else: return u""
    if "-L" in flags:
        if len(grepLines) == 0:
            return fileName
        else: return u""
    fileData = getFileContents(fileName)
    if "-Unicode-lines" in flags:
        allLines = fileData.splitlines(True)
    else:
        allLines = [l + "\n" for l in fileData.split('\n')]
    if "-v" in flags:
        fileLines = len(allLines)
        invLines = []
        for i in range(fileLines):
            if not i in grepLines:
                invLines.append(i)
        grepLines = invLines
    if "-m" in flags:
        maxcount = int(flags["-m"])
        if maxcount < len(grepLines):
            grepLines = grepLines[0:maxcount]
    if "-c" in flags:
        return u"%i" % len(grepLines)
    result = u""
    for matchedLine in grepLines:
        if "-n" in flags:
            result += u"%i:" % matchedLine
        result += allLines[matchedLine-1]
    if len(result) > 0 and result[-1] == u"\n": result = result[:-1]
    return result

def expected_grep_results(fileName, grepLines, flags):
    if "-l" in flags:
        if len(grepLines) > 0:
            return fileName
        else: return u""
    if "-L" in flags:
        if len(grepLines) == 0:
            return fileName
        else: return u""
    fileData = getFileContents(fileName)
    if "-Unicode-lines" in flags:
        allLines = fileData.splitlines(True)
    else:
        if len(fileData) > 0 and fileData[-1] == "\n":
            fileData = fileData[:-1]
        allLines = [l + "\n" for l in fileData.split('\n')]
    if "-v" in flags:
        fileLines = len(allLines)
        invLines = []
        for i in range(1, fileLines+1):
            if not i in grepLines:
                invLines.append(i)
        grepLines = invLines
    if "-m" in flags:
        maxcount = int(flags["-m"])
        if maxcount < len(grepLines):
            grepLines = grepLines[0:maxcount]
    if "-c" in flags:
        return u"%i" % len(grepLines)
    result = u""
    for matchedLine in grepLines:
        if "-n" in flags:
            result += u"%i:" % matchedLine
        result += allLines[matchedLine-1]
    if len(result) > 0 and result[-1] == u'\n':
        result = result[:-1]
    return result

def end_element_close_file(name):
    global outf
    global outfpath
    global in_datafile
    global dataFileName
    if name == 'datafile' and in_datafile:
        outf.close()
        os.chmod(outfpath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP | stat.S_IROTH | stat.S_IWOTH)
        in_datafile = False

def make_data_files(greptest_xml):
    p = xml.parsers.expat.ParserCreate()
    p.StartElementHandler = start_element_open_file
    p.CharacterDataHandler = char_data_write_contents
    p.EndElementHandler = end_element_close_file
    p.Parse(greptest_xml, 1)

def escape_quotes(e):  return e.replace(u"'", u"'\\''")

failure_count = 0

def execute_grep_test(flags, regexp, datafile, expected_result):
    global failure_count
    flag_string = ""
    for f in flags:
        if flag_string != "": flag_string += u" "
        if flags[f] == True: flag_string += f
        else: flag_string += f + "=" + flags[f]
    grep_cmd = u"%s %s '%s' %s" % (grep_program_under_test, flag_string, escape_quotes(regexp), os.path.join(options.datafile_dir, datafile))
    if options.verbose:
        print("Doing: " + grep_cmd)
    try:
        grep_out = codecs.decode(subprocess.check_output(grep_cmd.encode('utf-8'), cwd=options.exec_dir, shell=True), 'utf-8')
    except subprocess.CalledProcessError as e:
        grep_out = codecs.decode(e.output, 'utf-8')
    if len(grep_out) > 0 and grep_out[-1] == '\n': grep_out = grep_out[:-1]
    if grep_out != expected_result:
        print(u"Test failure: {%s} expecting {%s} got {%s}" % (grep_cmd, expected_result, grep_out))
        failure_count += 1
    else:
        if options.verbose:
            print(u"Test success: regexp {%s} on datafile {%s} expecting {%s} got {%s}" % (regexp, datafile, expected_result, grep_out))

flag_map = {'-CarryMode' : ['Compressed', 'BitBlock'],
            '-v' : [],
            '-n' : [],
            '-m' : ['2', '5'],
            '-DisableMatchStar' : [],
            '-ccc-type' : ['ternary'],
            '-EnableTernaryOpt' : []}

def add_random_flags(flags):
    for i in range(options.random_flag_count):
        rand_flag = flag_map.keys()[random.randint(0, len(flag_map) - 1)]
        values = flag_map[rand_flag]
        if values == []:
            flags[rand_flag] = True
        else:
            flags[rand_flag] = values[random.randint(0, len(values) - 1)]

def start_element_do_test(name, attrs):
    if name == 'grepcase':
        if not 'regexp' in attrs or not 'datafile' in attrs:
            print("Bad grepcase: missing regexp and/or datafile attributes.")
            return
        flags = {}
        if 'flags' in attrs:
            for field in attrs['flags'].split(' '):
                flag_and_value = field.split('=')
                flag = flag_and_value[0]
                if len(flag_and_value) == 1:
                    flags[flag] = True
                else: flags[flag] = flag_and_value[1]
        else:
            add_random_flags(flags)
        if 'grepcount' in attrs:
            flags["-c"] = True
            expected_result = attrs['grepcount']
            if "-m" in flags:
                if int(flags["-m"]) < int(attrs['grepcount']):
                    expected_result = flags["-m"]
        elif 'greplines' in attrs:
            if attrs['greplines'] == '':
                expected_result = expected_grep_results(attrs['datafile'], [], flags)
            else:
                lineFields = attrs['greplines'].split(' ')
                lines = [int(f) for f in lineFields]
                expected_result = expected_grep_results(attrs['datafile'], lines, flags)
        execute_grep_test(flags, attrs['regexp'], attrs['datafile'], expected_result)

def run_tests(greptest_xml):
    global failure_count
    p = xml.parsers.expat.ParserCreate()
    p.StartElementHandler = start_element_do_test
    p.Parse(greptest_xml, 1)
    if failure_count > 0: exit(1)

if __name__ == '__main__':
    QA_dir = os.path.dirname(sys.argv[0])
    option_parser = optparse.OptionParser(usage='python %prog [options] <grep_executable>', version='1.0')
    option_parser.add_option('-d', '--datafile_dir',
                          dest = 'datafile_dir', type='string', default='testfiles',
                          help = 'directory for test files.')
    option_parser.add_option('-t', '--testcases',
                          dest = 'testcases', type='string', default='greptest.xml',
                          help = 'grep test case file (XML format).')
    option_parser.add_option('-e', '--exec_dir',
                          dest = 'exec_dir', type='string', default='.',
                          help = 'executable directory')
    option_parser.add_option('--random_seed',
                          dest = 'random_seed', type='int', default=47367,
                          help = 'random number seed')
    option_parser.add_option('--random_flag_count',
                          dest = 'random_flag_count', type='int', default=0,
                          help = 'number of random flags to add')
    option_parser.add_option('-v', '--verbose',
                          dest = 'verbose', action='store_true', default=False,
                          help = 'verbose output: show successful tests')
    option_parser.add_option('-U', '--UTF-16',
                          dest = 'utf16', action='store_true', default=False,
                          help = 'test UTF-16 processing')
    options, args = option_parser.parse_args(sys.argv[1:])
    if len(args) != 1:
        option_parser.print_usage()
        sys.exit(1)
        if not os.path.exists(options.datafile_dir):
            os.mkdir(options.datafile_dir)
        if not os.path.isdir(options.datafile_dir):
            print("Cannot use %s as working test file directory.\n" % options.datafile_dir)
            sys.exit(1)
    random.seed(options.random_seed)
    grep_program_under_test = args[0]
    grep_test_file = open(os.path.join(QA_dir,options.testcases), 'r')
    grep_test_spec = grep_test_file.read()
    grep_test_file.close()

    make_data_files(grep_test_spec)
    run_tests(grep_test_spec)
