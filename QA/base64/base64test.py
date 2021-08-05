import sys, subprocess, optparse, os, string, random
import codecs
import xml.etree.ElementTree as ET
import base64

global options
global base64_test_file_name
failure_count = 0

def generateTestFile(name, content):
    if content == None or len(content) == 0:
        return
    outfpath = os.path.join(options.datafile_dir, name)
    outf = codecs.open(outfpath, encoding='utf-8', mode='w')
    outf.write(content)
    outf.close()

def generateRandomStr(length):
    outputStr = ""
    for i in range(length):
        outputStr += random.choice(string.printable)
        # outputStr += random.choice(string.ascii_letters + "\n")
    return outputStr

def generateRandomTestFile(name, length):
    content = generateRandomStr(length)
    generateTestFile(name, content)

def make_data_files(base64test_xml):
    tree = ET.parse(base64test_xml)
    root = tree.getroot()
    for child in root:
        if child.tag == 'testcase':
            if "name" in child.attrib:
                generateTestFile(child.attrib["name"], child.text)
        elif child.tag == 'random-testcase':
            if "length" in child.attrib and "name" in child.attrib:
                generateRandomTestFile(child.attrib["name"], int(child.attrib["length"]))

def run_test(test_name):
    global options
    global failure_count

    test_file = os.path.join(options.datafile_dir, test_name)
    base64_cmd = "%s %s" % (base64_program_under_test, test_file)
    if options.verbose:
        print("Doing: " + base64_cmd)

    try:
        base64_out = subprocess.check_output(base64_cmd.encode('utf-8'), cwd=options.exec_dir, shell=True)
    except subprocess.CalledProcessError as e:
        base64_out = e.output

    inf = open(test_file, mode='rb')
    content = inf.read()
    inf.close()
    expectedOutput = base64.b64encode(content)
    if expectedOutput != base64_out:
        print("Test failure: datafile {%s}" % (test_name))
        failure_count += 1
    else:
        if options.verbose:
            print("Test success: datafile {%s}" % (test_name))

def run_tests(base64test_xml):
    tree = ET.parse(base64test_xml)
    root = tree.getroot()
    for child in root:
        if child.tag == 'testcase':
            if "name" in child.attrib:
                run_test(child.attrib["name"])
        elif child.tag == 'random-testcase':
            if "length" in child.attrib and "name" in child.attrib:
                run_test(child.attrib["name"])
    if failure_count > 0: exit(1)

if __name__ == '__main__':
    global options
    global base64_test_file_name
    QA_dir = os.path.dirname(sys.argv[0])
    option_parser = optparse.OptionParser(usage='python %prog [options] <base64_executable>', version='1.0')
    option_parser.add_option('-d', '--datafile_dir',
                             dest = 'datafile_dir', type='string', default='testfiles',
                             help = 'directory for test files.')
    option_parser.add_option('-t', '--testcases',
                             dest = 'testcases', type='string', default='base64test.xml',
                             help = 'base64 test case file (XML format).')
    option_parser.add_option('-e', '--exec_dir',
                             dest = 'exec_dir', type='string', default='.',
                             help = 'executable directory')
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
    base64_program_under_test = args[0]
    base64_test_file_name = os.path.join(QA_dir,options.testcases)
    make_data_files(base64_test_file_name)
    run_tests(base64_test_file_name)

