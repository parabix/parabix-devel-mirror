import character_deposit
import sys, subprocess, optparse, os, string, random
import codecs
import shutil

global options
global lz4d_program_under_test

failure_count = 0

def run_tests(test_dir, output_dir, python_output_dir):
    global options
    global failure_count

    test_files = os.listdir(test_dir)
    for test_file in test_files:
        if not test_file.endswith('.txt'):
            continue
        output_file = "out_" + test_file

        test_file_full_path = os.path.join(test_dir, test_file)
        output_file_full_path = os.path.join(output_dir, output_file)
        python_output_file_full_path = os.path.join(python_output_dir, output_file)

        character_deposit_cmd = "%s b %s %s" % (lz4d_program_under_test, test_file_full_path, output_file_full_path)

        if options.verbose:
            print("Doing: " + character_deposit_cmd)

        try:
            character_deposit_out = subprocess.check_output(character_deposit_cmd.encode('utf-8'), cwd=options.exec_dir, shell=True)
        except subprocess.CalledProcessError as e:
            character_deposit_out = e.output

        character_deposit.handle_deposit(test_file_full_path, python_output_file_full_path, 'b')

        with codecs.open(output_file_full_path, mode='rb') as inf:
            output_content = inf.read()
        with codecs.open(python_output_file_full_path, mode='rb') as inf:
            expected_output_content = inf.read()

        if output_content != expected_output_content:
            print("Test failure: datafile {%s}" % test_file)
            failure_count += 1
        else:
            if options.verbose:
                print("Test success: datafile {%s}" % test_file)

    if failure_count > 0:
        print("Test Failure")
        exit(1)
    else:
        print("Test Success")


if __name__ == '__main__':
    QA_dir = os.path.dirname(sys.argv[0])

    option_parser = optparse.OptionParser(usage='python %prog [options] <character_deposit_executable>', version='1.0')
    option_parser.add_option('-d', '--datafile_dir',
                             dest='datafile_dir', type='string', default='testfiles',
                             help='directory for test files.')
    option_parser.add_option('-e', '--exec_dir',
                             dest='exec_dir', type='string', default='.',
                             help='executable directory')
    option_parser.add_option('-o', '--output_dir',
                             dest='output_dir', type='string', default='output',
                             help='output directory of decompressed files')
    option_parser.add_option('-p', '--python_output_dir',
                             dest='python_output_dir', type='string', default='python_output',
                             help='output directory of expected result (python version)')
    option_parser.add_option('-v', '--verbose',
                             dest='verbose', action='store_true', default=False,
                             help='verbose output: show successful tests')
    options, args = option_parser.parse_args(sys.argv[1:])

    if len(args) != 1:
        option_parser.print_usage()
        sys.exit(1)

    if not os.path.exists(options.datafile_dir):
        print(options.datafile_dir + ' not exists')
        sys.exit(1)

    if os.path.exists(options.output_dir):
        shutil.rmtree(options.output_dir)
    os.mkdir(options.output_dir)


    if os.path.exists(options.python_output_dir):
        shutil.rmtree(options.python_output_dir)
    os.mkdir(options.python_output_dir)

    lz4d_program_under_test = args[0]

    test_dir = os.path.join(QA_dir, options.datafile_dir)
    output_dir = os.path.join(QA_dir, options.output_dir)
    python_output_dir = os.path.join(QA_dir, options.python_output_dir)
    run_tests(test_dir, output_dir, python_output_dir)

