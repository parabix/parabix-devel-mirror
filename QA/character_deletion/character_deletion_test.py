import sys, subprocess, optparse, os, string, random
import codecs
import xml.etree.ElementTree as ET

global options
global character_deletion_test_file_name
global character_deletion_program_under_test
failure_count = 0

def generateRandomTestFile(name, length):
    content = generate_random_str(length)
    generate_test_file(name, content)


def make_data_files(deletion_test_xml):
    tree = ET.parse(deletion_test_xml)
    root = tree.getroot()
    for child in root:
        if child.tag == 'testcase':
            if "name" in child.attrib:
                generate_test_file(child.attrib["name"], child.text)
        elif child.tag == 'random-testcase':
            if "length" in child.attrib and "name" in child.attrib:
                generateRandomTestFile(child.attrib["name"], int(child.attrib["length"]))


def run_test(test_name, random_deletion, character_to_be_deleted):
    global options
    global failure_count

    test_file = os.path.join(options.datafile_dir, test_name)
    inf = codecs.open(test_file, encoding='utf-8', mode='r')
    content = inf.read()
    inf.close()

    target_character = None
    if random_deletion:
        target_character = random.choice(string.ascii_letters)
    else:
        target_character = character_to_be_deleted

    expected_output = content.replace(target_character, "")

    character_deletion_cmd = "%s %s %s" % (character_deletion_program_under_test, target_character, test_file)
    if options.verbose:
        print("Doing: " + character_deletion_cmd)
    try:
        deletion_out = subprocess.check_output(character_deletion_cmd.encode('utf-8'), cwd=options.exec_dir, shell=True)
    except subprocess.CalledProcessError as e:
        deletion_out = e.output

    if expected_output != deletion_out:
        print("Test failure: datafile {%s}, deletion character {%s}" % (test_name, target_character))
        failure_count += 1
    else:
        if options.verbose:
            print("Test success: datafile {%s}, deletion character {%s}" % (test_name, target_character))


def run_tests(base64test_xml):
    tree = ET.parse(base64test_xml)
    root = tree.getroot()
    for child in root:

        if "name" in child.attrib:
            test_name = child.attrib["name"]
            random_deletion = True
            character_to_be_deleted = None
            if 'random-deletion' in child.attrib:
                random_deletion = child.attrib['random-deletion'] == 'true'
            if 'character' in child.attrib:
                character_to_be_deleted = child.attrib['character']
            run_test(test_name, random_deletion, character_to_be_deleted)

    if failure_count > 0:
        exit(1)


def generate_test_file(name, content):
    if not content or len(content) == 0:
        return
    outfpath = os.path.join(options.datafile_dir, name)
    outf = codecs.open(outfpath, encoding='utf-8', mode='w')
    outf.write(content)
    outf.close()


def generate_random_str(length):
    outputStr = ""
    for i in range(length):
        # outputStr += random.choice(string.printable)
        outputStr += random.choice(string.ascii_letters + "\n")
    return outputStr


if __name__ == '__main__':
    global options
    global character_deletion_test_file_name
    global character_deletion_program_under_test

    QA_dir = os.path.dirname(sys.argv[0])
    option_parser = optparse.OptionParser(usage='python %prog [options] <character_deletion_executable>', version='1.0')
    option_parser.add_option('-d', '--datafile_dir',
                             dest='datafile_dir', type='string', default='testfiles',
                             help='directory for test files.')
    option_parser.add_option('-t', '--testcases',
                             dest='testcases', type='string', default='character_deletion_test.xml',
                             help='character_deletion test case file (XML format).')
    option_parser.add_option('-e', '--exec_dir',
                             dest='exec_dir', type='string', default='.',
                             help='executable directory')
    option_parser.add_option('-v', '--verbose',
                             dest='verbose', action='store_true', default=False,
                             help='verbose output: show successful tests')
    options, args = option_parser.parse_args(sys.argv[1:])

    if len(args) != 1:
        option_parser.print_usage()
        sys.exit(1)

    if not os.path.exists(options.datafile_dir):
        os.mkdir(options.datafile_dir)

    if not os.path.isdir(options.datafile_dir):
        print("Cannot use %s as working test file directory.\n" % options.datafile_dir)
        sys.exit(1)

    character_deletion_program_under_test = args[0]
    character_deletion_test_file_name = os.path.join(QA_dir, options.testcases)
    make_data_files(character_deletion_test_file_name)
    run_tests(character_deletion_test_file_name)
