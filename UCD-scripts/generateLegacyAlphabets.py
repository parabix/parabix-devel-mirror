#
# Legacy Character Set Definitions
#

from string import Template
import cformat

Alphabet_Template = r"""//
// ${alphabet_name}.cpp - Legacy character set as UnicodeMappableAlphabet
//
namespace ${alphabet_name} {
std::vector<codepoint_t> codepoints = 
    {${codepoint_list}};

UnicodeMappableAlphabet alphabet("${alphabet_name}", 128, &codepoints);

}
"""


#
# Parse a WHATWG specification of a single-byte-encoding, return 
# a dictionary mapping "pointers" to code points.
#
def parse_WHATWG_single_byte_encoding(enc_name):
    f = open("encodings/WHATWG/index-" + enc_name + ".txt")

    # To find the pointers and their corresponding code points in an index,
    # let lines be the result of splitting the resource's contents on U+000A.
    lines = f.read().split(chr(0x0A))
    f.close()
    # Then remove each item in lines that is the empty string or starts with U+0023.
    lines = [line for line in lines if line != "" and line[0] != chr(0x23)]
    # Then the pointers and their corresponding code points are found by splitting each item in lines on U+0009.
    # The first subitem is the pointer (as a decimal number) and the second is the corresponding code point 
    # (as a hexadecimal number). Other subitems are not relevant.
    index = {}
    for line in lines:
        fields = line.split(chr(0x09))
        pointer = int(fields[0])
        cp = int(fields[1], 16)
        index[pointer] = cp
    return index

def validate_full_extended_ASCII(idx):
    if len(idx.keys()) != 128: return False
    for k in range(128):
        if not k in idx: return False
        if idx[k] < 0 or idx[k] > 0x10FFFF: return False
    return True


def make_encoder(enc_name):
    idx = parse_WHATWG_single_byte_encoding(enc_name)
    if not validate_full_extended_ASCII(idx):
        print(enc_name + " is not a full extended ASCII single-byte-encoding")
        return
    cps = [idx[k] for k in range(128)]
    cp_list = cformat.multiline_fill(['0x%04x' % cp for cp in cps], ',', 8)
    return Template(Alphabet_Template).substitute(alphabet_name = enc_name.replace('-','_'), codepoint_list = cp_list)

