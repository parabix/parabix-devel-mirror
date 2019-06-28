#!/usr/bin/env python3

# Note: this file must be run from the QA/xml directory

import binascii
import lxml.etree
import os
import shutil
import sys
from string import *
from subprocess import *

xml_exe = ""

xmlconf_home = "./xmlconf"
output_home = "./out"
output_messages_dir = output_home + "/msg"
output_file_dir = output_home + "/file"
expected_message_dir = "./ExpectedOutput/Messages"
testcases = []

verbose = len(sys.argv) == 3 and (sys.argv[2] == '-v' or sys.argv[2] == '--verbose')


def parse_xmlconf(xmlconf):
    testcase_base = ""
    for event, element in lxml.etree.iterparse(xmlconf, events=("start",)):
        if element.tag == "TESTCASES":
            if element.attrib.has_key("{http://www.w3.org/XML/1998/namespace}base"):
                testcase_base = "/" + element.get("{http://www.w3.org/XML/1998/namespace}base")
        elif element.tag == "TEST":
            if element.attrib.has_key("RECOMMENDATION"):
                if element.get("RECOMMENDATION") == "XML1.1":
                    continue
            if element.attrib.has_key("URI"):
                uri = element.get("URI")
                addr = uri.split('/')
                filename = addr[len(addr)-1]
                subdir = uri[:uri.find(filename)]
                add_testcase(testcase_base, subdir, filename)


def add_testcase(testsuit_dir, subdir, filename):
    global testcases
    full_dir = testsuit_dir + subdir
    file = open(xmlconf_home + full_dir + filename, 'rb')
    filedata = file.read()
    if filedata.find(b'DOCTYPE') == -1:
        e = binascii.hexlify(filedata[:2])
        if e != b'0000' and e != b'feff' and e != b'fffe' and e != b'003c' and e != b'3c00' and e != b'4c6f':
            testcases.append((full_dir, filename))


def invoke_parabix_xml_executable(xmlfile, out_f, err_f):
    call([xml_exe, xmlfile], stdout=out_f, stderr=err_f)


def run_tests():
    test_num = 1
    for testcase in testcases:
        msg_dir = output_messages_dir + '/' + testcase[0]
        out_dir = output_file_dir + '/' + testcase[0]
        if not os.path.exists(msg_dir):
            os.makedirs(msg_dir)
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        if verbose:
            print("Running Test #" + str(test_num) + ": " + testcase[0] + testcase[1])
            test_num += 1
        msg_f = open(msg_dir + testcase[1], 'w')
        out_f = open(out_dir + testcase[1], 'w')
        testfile = xmlconf_home + testcase[0] + testcase[1]
        invoke_parabix_xml_executable(testfile, out_f, msg_f)
        msg_f.close()
        out_f.close()


if __name__ == "__main__":
    # get executable
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print("USAGE: python3 run_test_suite.py <xml exe> [-v]")
        exit(1)
    xml_exe = sys.argv[1]
    # setup output directories
    if os.path.exists(output_home + ".bak"):
        shutil.rmtree(output_home + ".bak")
    if os.path.exists(output_home):
        shutil.move(output_home, output_home + ".bak")
    os.mkdir(output_home)
    os.mkdir(output_file_dir)
    os.mkdir(output_messages_dir)

    parse_xmlconf(xmlconf_home + "/xmlconf.xml")
    print("-- Found " + str(len(testcases)) + " test cases")
    print("-- Begining tests...")
    run_tests()
    print("-- Tests finished, begining validation...")
    test_num = 0
    failed_count = 0
    for testcase in testcases:
        test_num += 1
        expected_out_f = open(expected_message_dir + testcase[0] + testcase[1], 'r')
        actual_out_f = open(output_messages_dir + testcase[0] + testcase[1], 'r')
        expected_out_data = expected_out_f.read()
        actual_out_data = actual_out_f.read()
        if (expected_out_data != actual_out_data):
            failed_count += 1
            print("Test #" + str(test_num) + ": \u001b[31mFailed\u001b[0m: " + testcase[0] + testcase[1])
            print("    Expected Message: \"" + expected_out_data.rstrip('\r\n') + "\"")
            print("    Actual Message:   \"" + actual_out_data.rstrip('\r\n') + "\"")
        elif verbose:
            print("Test #" + str(test_num) + ": \u001b[32mPassed\u001b[0m")

    if failed_count == 0:
        print("\n\u001b[32mPassed\u001b[0m all " + str(len(testcases)) + " tests")
    else:
        print("\n\u001b[31mFailed\u001b[0m " + str(failed_count) + " of " + str(len(testcases)) + " tests")
