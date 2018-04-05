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
# <grepcase regexp="in" datafile="simple1" grepcount="2"/>
# <grepcase regexp="[A-Z]" datafile="simple1" grepcount="1"/>
#
# </greptest>


import sys, subprocess, os, optparse, re, codecs, stat
import xml.parsers.expat
import sys
import codecs

sys.stdout = codecs.getwriter('utf8')(sys.stdout)
sys.stderr = codecs.getwriter('utf8')(sys.stderr)

in_datafile = False

def start_element_open_file(name, attrs):
	global outf
	global outfpath
	global in_datafile
	if name == 'datafile':
		idFound = False
		for a in attrs:
			if a == 'id':
				filename = attrs[a]
				idFound = True
		if not idFound:
			print("Expecting id attribute for datafile, but none found.")
			exit(-1)
                outfpath = os.path.join(options.datafile_dir, filename)
		if options.utf16: outf = codecs.open(outfpath, encoding='utf-16BE', mode='w')
		else: outf = codecs.open(outfpath, encoding='utf-8', mode='w')
		in_datafile = True

def char_data_write_contents(data):
	if in_datafile:
		outf.write(data)

def end_element_close_file(name):
	global outf
	global outfpath
	global in_datafile
	if name == 'datafile':
		outf.close()
                os.chmod(outfpath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP | stat.S_IROTH | stat.S_IWOTH)
		in_datafile = False

def make_data_files(greptest_xml):
	p = xml.parsers.expat.ParserCreate()
	p.StartElementHandler = start_element_open_file
	p.CharacterDataHandler = char_data_write_contents
	p.EndElementHandler = end_element_close_file
	p.Parse(greptest_xml, 1)

def escape_quotes(e):  return e.replace("'", "'\\''")


failure_count = 0

def start_element_do_test(name, attrs):
        global failure_count
	if name == 'grepcase':
		regexp = None
		datafile = None
		expected_count = None
		flags = None
		for a in attrs:
			if a == 'regexp':
				regexp = attrs[a]
			elif a == 'datafile':
				datafile = attrs[a]
			elif a == 'grepcount':
				expected_count = attrs[a]
			elif a == 'flags':
				flags = attrs[a]
		if regexp == None or datafile == None or expected_count == None:
			print("Bad grepcase: missing regexp and/or datafile attributes.")
			return
		if flags == None: flags = "-c" # Our default is counting if flags not set explicitly
		#execute grep test
                grep_cmd = "%s %s '%s' %s" % (grep_program_under_test, flags, escape_quotes(regexp), os.path.join(options.datafile_dir, datafile))
                if options.verbose:
                    print("Doing: " + grep_cmd)
		try:
                    grep_out = subprocess.check_output(grep_cmd.encode('utf-8'), cwd=options.exec_dir, shell=True)
                except subprocess.CalledProcessError, e:
                    grep_out = e.output
		if len(grep_out) > 0 and grep_out[-1] == '\n': grep_out = grep_out[:-1]
		m = re.search('[0-9]+', grep_out)
		if m == None or m.group(0) != expected_count:
			print("Test failure: {%s} expecting {%s} got {%s}" % (grep_cmd, expected_count, grep_out))
                        failure_count += 1
		else:
			if options.verbose:
				print("Test success: regexp {%s} on datafile {%s} expecting {%s} got {%s}" % (regexp, datafile, expected_count, grep_out))

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
	grep_program_under_test = args[0]
	grep_test_file = open(os.path.join(QA_dir,options.testcases), 'r')
	grep_test_spec = grep_test_file.read()
	grep_test_file.close()

	make_data_files(grep_test_spec)
	run_tests(grep_test_spec)


