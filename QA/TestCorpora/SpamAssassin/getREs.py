import re, sys

header_exists_RE = re.compile('^ *header .* exists:([-A-Za-z0-9:]+) *(?:#.*)?$')

header_match_RE1 = re.compile('^ *header .* ([-A-Za-z0-9:]+) *=~ *(/)(.*)/([misx]*) *(?:#.*)?$')
header_match_RE2 = re.compile('^ *header .* ([-A-Za-z0-9:]+) *=~ *m([{])(.*)[}]([misx]*) *(?:#.*)?$')
header_match_RE3 = re.compile('^ *header .* ([-A-Za-z0-9:]+) *=~ *m([^A-Za-z0-9])(.*)\1([misx]*) *(?:#.*)?$')

# These are currently ignored
header_nomatch_RE1 = re.compile('^ *header .* ([-A-Za-z0-9:]+) *=!')


pattern_RE1 = re.compile(r"""[^\\]*(/)(.*)/([misx]*) *(?:#.*)?$""")
pattern_RE2 = re.compile('.*m([{])(.*)[}]([misx]*) *(?:#.*)?$')
pattern_RE3 = re.compile('.*m([^A-Za-z0-9])(.*)\1([misx]*) *(?:#.*)?$')



def report_pattern(p):
    sys.stdout.write(p + "\n")

def get_patterns(fileName):
    f = open(fileName)
    lines = f.readlines()
    for t in lines:
        match = header_exists_RE.match(t)
        if match:
            report_pattern("^" + match.group(1))
            continue
        header_match = header_match_RE1.match(t)
        if not header_match: header_match = header_match_RE2.match(t) # Try m{...}  
        if not header_match: header_match = header_match_RE3.match(t) # Try m syntax with other delimiters  
        if header_match:
            join = ":.*"
            pattern = header_match.group(3)
            if pattern[0] == '^':
                join = ": *"
                pattern = pattern[1:]
            if header_match.group(4) != '':
                pattern = '(?' + header_match.group(4) + ')' + pattern
            if header_match.group(1) == 'ToCc':
                report_pattern("^To:" + join + pattern)
                report_pattern("^Cc:" + join + pattern)
                continue
            if header_match.group(1) == 'ALL':
                report_pattern("^[-A-Za-z0-9]+" + join + pattern)
                continue
            report_pattern("^" + header_match.group(1) + join + pattern)
        match = pattern_RE1.match(t)
        if not match: match = pattern_RE2.match(t) # Try m{...}  
        if not match: match = pattern_RE3.match(t) # Try m syntax with other delimiters
        if not match: continue
        pattern = match.group(2)
        if match.group(3) != '':
            pattern = '(?' + match.group(3) + ')' + pattern
        report_pattern(pattern)
    f.close()

if __name__ == "__main__":
    get_patterns(sys.argv[1])
