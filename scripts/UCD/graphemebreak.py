#
# -*- coding: UTF-8
#
# Robert D. Cameron
#
# Licensed under Open Software License 3.0.
#
#
import string, os.path, re, random
import UCD_config

version_regexp = re.compile(".*Version\s+([0-9.]*)\s+of the Unicode Standard.*")

def setVersionfromReadMe_txt():
    f = open(UCD_config.UCD_src_dir + "/" + 'ReadMe.txt')
    lines = f.readlines()
    for t in lines:
        m = version_regexp.match(t)
        if m:
            UCD_config.version = m.group(1)
            print("Version %s" % m.group(1))

#
#  Processing files of the UCD
#
#  General format for skippable comments, blank lines
UCD_skip = re.compile("^#.*$|^\s*$")

#
#  Grapheme Break Format
Grapheme_break_regexp = re.compile("^÷\s*((?:[0-9A-F]+\s*[÷×]\s*)+)\s*#.*$")


def parse_grapheme_break_txt():
    f = open(UCD_config.UCD_src_dir + "/auxiliary/" + 'GraphemeBreakTest.txt')
    lines = f.readlines()
    test_dict = {}
    for t in lines:
        if UCD_skip.match(t): continue  # skip comment and blank lines
        m = Grapheme_break_regexp.match(t)
        if m:
            codepoints = re.findall("[0-9A-F]+", m.group(1))
            if m.group(1) in test_dict: continue
            if len(codepoints) == 0: continue
            if '000D' in codepoints or '000A' in codepoints or '0001' in codepoints:
                continue
            breaksyms = re.findall("[÷×]", m.group(1))
            if breaksyms[-1] != '÷':
                print("Unexpected test case:", codepoints, breaksyms)
            boundaries = [sym == "÷" for sym in breaksyms]
            test_dict[m.group(1)] = (codepoints, boundaries)
    return [test_dict[k] for k in sorted(test_dict.keys())]

def xml_test_case_data(tests):
    data = ""
    for t in tests:
        (cps, syms) = t
        for cp in cps:
            data += "&#x" + cp + ";"
        data += "\n"
    return data

grepcase_template = r"""<grepcase regexp="%s" datafile="graphemebreaktests" greplines="%s"/>
"""

def break_exp(boundary):
    if boundary: return "\\b{g}"
    return "\\B{g}"

def grepcase_pair(line_no, codepoints, boundaries):
    cp_classes = ["\\u{" + cp + "}" for cp in codepoints]
    required = "^"
    for i in range(len(codepoints)):
        required += cp_classes[i] + break_exp(boundaries[i])
    required += "$"
    failure_posn = random.randint(0, len(codepoints) - 1)
    failure = ""
    for i in range(len(codepoints)):
        failure += cp_classes[i]
        if i == failure_posn:
            failure += break_exp(not boundaries[i])
    failure += "$"
    return grepcase_template % (required, repr(line_no)) + grepcase_template % (failure, "")


def grepcases1(tests):
    cases = ""
    for i in range(len(tests)):
        (cps, syms) = tests[i]
        cases += grepcase_pair(i+1, cps, syms)


    return cases

def grepcases2(tests):
    by_grapheme_count = {}
    for i in range(len(tests)):
        (cps, syms) = tests[i]
        line_no = i + 1
        grapheme_count = len([x for x in syms if x])
        if grapheme_count in by_grapheme_count:
            by_grapheme_count[grapheme_count].append(line_no)
        else:
            by_grapheme_count[grapheme_count] = [line_no]
    cases = ""
    for c in sorted(by_grapheme_count.keys()):
        expr = "^" + ("\\X" * c) + "$"
        lines = " ".join([repr(line_no) for line_no in by_grapheme_count[c]])
        cases += grepcase_template % (expr, lines)
    return cases



grapheme_test_xml_template = r"""<greptest>
<datafile id="graphemebreaktests">%s</datafile>
%s
</greptest>
"""

if __name__ == "__main__":
    tests = parse_grapheme_break_txt()
    #print(xml_test_case_data(tests))
    test_file = grapheme_test_xml_template % (xml_test_case_data(tests), grepcases1(tests) + grepcases2(tests))
    f = open('grapheme_test.xml', 'w')
    f.write(test_file)
    f.close()







