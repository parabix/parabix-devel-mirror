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
import locale

sys.stdout = codecs.getwriter('utf8')(sys.stdout)
sys.stderr = codecs.getwriter('utf8')(sys.stderr)

in_datafile = False
dataFileName = ""

fileContents = {}

def getFileContents(fileName):
    if not fileName in fileContents:
        outfpath = os.path.join(options.datafile_dir, fileName)
        f = codecs.open(outfpath, encoding='utf-16le', mode='r')
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
        if options.utf16: outf = codecs.open(outfpath, encoding='utf-16le', mode='w')
        else: outf = codecs.open(outfpath, encoding='utf-16le', mode='w')
        in_datafile = True

def char_data_write_contents(data):
    global in_datafile
    if in_datafile:
        outf.write(data)

def expected_grep_results(fileName, grepLines, flags):
    fileData = getFileContents(fileName)
    if "-Unicode-lines" in flags and flags["-Unicode-lines"] != 0:
        allLines = fileData.splitlines(True)
    else:
        if len(fileData) == 0: allLines = []
        else:
            if fileData[-1] == "\n": fileData = fileData[:-1]
            allLines = [l + "\n" for l in fileData.split('\n')]
    if "-v" in flags:
        fileLines = len(allLines)
        invLines = []
        for i in range(1, fileLines+1):
            if not i in grepLines:
                invLines.append(i)
        grepLines = invLines
    if "-l" in flags:
        if len(grepLines) > 0:
            return os.path.join(options.datafile_dir, fileName)
        else: return u""
    if "-L" in flags:
        if len(grepLines) == 0:
            return os.path.join(options.datafile_dir, fileName)
        else: return u""
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
    outfpath = os.path.join(options.datafile_dir, datafile)
    grep_cmd = u"%s %s '%s' %s" % (grep_program_under_test, flag_string, escape_quotes(regexp), outfpath)
    if options.verbose:
        print("Doing: " + grep_cmd)
    try:
        grep_out = subprocess.check_output(grep_cmd, shell=True) 
    except subprocess.CalledProcessError as e:
        grep_out = codecs.decode(e.output, 'utf-8')
    if "-c" not in flags:
        testpath = os.path.join(options.datafile_dir, 'test_output.txt')
        f = codecs.open(testpath, mode='w+')
        f.write(grep_out)
        f.close()
        f = codecs.open(testpath, encoding='utf-16le', mode='r')
        grep_out = f.read()
        f.close()
    #print(":".join("{:x}".format(ord(c)) for c in expected_result), "expected_result")

    if len(grep_out) > 1:
        if grep_out[-2] == '\n':
            grep_out = grep_out[:-2]
        if grep_out[-1] == '\n':
            grep_out = grep_out[:-1]
    if grep_out != expected_result:
        print(u"Test failure: {%s} expecting {%s} got {%s}" % (grep_cmd, expected_result, grep_out))
        failure_count += 1
    else:
        if options.verbose:
            print(u"Test success: regexp {%s} on datafile {%s} expecting {%s} got {%s}" % (regexp, datafile, expected_result, grep_out))

flag_map = {'-CarryMode' : ['Compressed', 'BitBlock'],
            'counting_choices' : ["-c", "-l", "-L"],
            '-v' : [],
            '-n' : [],
            '-m' : ['2', '5'],
            '-match-coordinates' : ['2', '8'],
            '-DisableMatchStar' : [],
            '-segment-size' : ['8192', '16384', '32768'],
            '-ccc-type' : ['ternary'],
            '-EnableTernaryOpt' : []}

def add_random_flags(flags, fileLength):
    selected = {}
    for i in range(options.random_flag_count):
        rand_flag = flag_map.keys()[random.randint(0, len(flag_map) - 1)]
        # Avoid duplicate flags and expensive test cases
        while rand_flag in selected or (rand_flag == "-v" and fileLength > 4000):
            rand_flag = flag_map.keys()[random.randint(0, len(flag_map) - 1)]
        selected[rand_flag] = True
        values = flag_map[rand_flag]
        if values == []:
            flags[rand_flag] = True
        else:
            choice = values[random.randint(0, len(values) - 1)]
            if rand_flag[0] == "-":
                flags[rand_flag] = choice
            else: flags[choice] = True

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
        if 'grepcount' in attrs:
            flags["-c"] = True
            expected_result = attrs['grepcount']
            if "-m" in flags:
                if int(flags["-m"]) < int(attrs['grepcount']):
                    expected_result = flags["-m"]
            execute_grep_test(flags, attrs['regexp'], attrs['datafile'], expected_result)
        else:
            if not 'greplines' in attrs:
                raise Exception('Expecting grepcount or greplines in grepcase')
            fileLength = len(getFileContents(attrs['datafile']))
            lines = []
            if attrs['greplines'] != '':
                lineFields = attrs['greplines'].split(' ')
                lines = [int(f) for f in lineFields]
            if len(flags) > 0:
                expected_result = expected_grep_results(attrs['datafile'], lines, flags)
                execute_grep_test(flags, attrs['regexp'], attrs['datafile'], expected_result)
            else:
                for i in range(options.tests_per_grepcase):
                    flags = {}
                    add_random_flags(flags, fileLength)
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
    option_parser.add_option('--tests_per_grepcase',
                          dest = 'tests_per_grepcase', type='int', default=1,
                          help = 'number of tests to generate per grepcase')
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
