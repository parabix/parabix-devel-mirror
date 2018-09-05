import lz4d

import sys, subprocess, optparse, os, string, random
import codecs
import shutil

global options
global lz4d_program_under_test

failure_count = 0

# For bitstream approach, there are still some bugs in some special small file
# (For now, it will have bug in 29k.txt.lz4)
test_options = [
    ('normal', '', {}),
]


def run_test(test_file, lz4_option, python_lz4_option, test_file_full_path, output_file_full_path, python_output_file_full_path):
    global failure_count
    lz4d_cmd = "%s %s --thread-num=1 -f %s %s" % (
    lz4d_program_under_test, lz4_option, test_file_full_path, output_file_full_path, )

    if options.verbose:
        print("Doing: " + lz4d_cmd)
    try:
        lz4d_out = subprocess.check_output(lz4d_cmd.encode('utf-8'), cwd=options.exec_dir, shell=True)
    except subprocess.CalledProcessError as e:
        lz4d_out = e.output

    lz4Decoder = lz4d.LZ4Decoder()

    lz4Decoder.decode(test_file_full_path, python_output_file_full_path, python_lz4_option)

    with codecs.open(output_file_full_path, mode='rb') as inf:
        output_content = inf.read()
    with codecs.open(python_output_file_full_path, mode='rb') as inf:
        expected_output_content = inf.read()

    if output_content != expected_output_content:
        print("Test failure: datafile {%s} option:{%s}" % (test_file, lz4_option))
        failure_count += 1
    else:
        if options.verbose:
            print("Test success: datafile {%s} option:{%s}" % (test_file, lz4_option))


def run_tests(test_dir, output_dir, python_output_dir):
    global options
    global failure_count

    test_files = os.listdir(test_dir)

    for test_file in test_files:
        if test_file.endswith('.lz4'):
            output_file = test_file.replace('.lz4', '')
            test_file_full_path = os.path.join(test_dir, test_file)
            for test_opt in test_options:
                output_file_full_path = os.path.join(output_dir, test_opt[0], output_file)
                python_output_file_full_path = os.path.join(python_output_dir, test_opt[0], output_file)
                run_test(test_file, test_opt[1], test_opt[2], test_file_full_path, output_file_full_path, python_output_file_full_path)

    if failure_count > 0:
        print("Test Failure")
        exit(1)
    else:
        print("Test Success")


if __name__ == '__main__':
    QA_dir = os.path.dirname(sys.argv[0])

    option_parser = optparse.OptionParser(usage='python %prog [options] <lz4d_ext_dep_executable>', version='1.0')
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
    for opt in test_options:
        os.mkdir(os.path.join(options.output_dir, opt[0]))

    if os.path.exists(options.python_output_dir):
        shutil.rmtree(options.python_output_dir)

    os.mkdir(options.python_output_dir)
    for opt in test_options:
        os.mkdir(os.path.join(options.python_output_dir, opt[0]))

    lz4d_program_under_test = args[0]

    test_dir = os.path.join(QA_dir, options.datafile_dir)
    output_dir = os.path.join(QA_dir, options.output_dir)
    python_output_dir = os.path.join(QA_dir, options.python_output_dir)
    run_tests(test_dir, output_dir, python_output_dir)

