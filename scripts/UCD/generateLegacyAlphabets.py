#
# Legacy Character Set Definitions
#

from string import Template
import cformat
import WHATWG_parser

Alphabet_Template = r"""//
// ${alphabet_name}.cpp - Legacy character set as UnicodeMappableAlphabet
//
namespace ${alphabet_name} {
std::vector<codepoint_t> codepoints =
    {${codepoint_list}};

UnicodeMappableAlphabet alphabet("${alphabet_name}", 128, &codepoints);

}
"""

def validate_full_extended_ASCII(idx):
    if len(idx.keys()) != 128: return False
    for k in range(128):
        if not k in idx: return False
        if idx[k] < 0 or idx[k] > 0x10FFFF: return False
    return True


def make_extended_ASCII_encoder(enc_name):
    idx = parse_WHATWG_index_file(enc_name)
    if not validate_full_extended_ASCII(idx):
        print(enc_name + " is not a full extended ASCII single-byte-encoding")
        return
    cps = [idx[k] for k in range(128)]
    cp_list = cformat.multiline_fill(['0x%04x' % cp for cp in cps], ',', 8)
    return Template(Alphabet_Template).substitute(alphabet_name = enc_name.replace('-','_'), codepoint_list = cp_list)

def GB_double_byte_table():
    idx = WHATWG_parser.parse_WHATWG_index_file('gb18030')
    tbl = "std::vector<std::vector<codepoint_t>> GB_DoubleByteTable = {\n"
    for byte1 in range(0x81, 0xFF):
        pointer_base = (byte1 - 0x81) * 190
        cps = [idx[p] for p in range(pointer_base, pointer_base + 190)]
        cp_list = cformat.multiline_fill(['0x%04x' % cp for cp in cps], ',', 5)
        tbl += "    {" + cp_list + "}"
        if byte1 != 0xFE:
            tbl += ",\n"
        else:
            tbl += "};\n"
    return tbl


def GB_range_table():
    idx = WHATWG_parser.parse_WHATWG_index_file('gb18030-ranges')
    tbl = "std::vector<std::pair<unsigned, unsigned>> GB_RangeTable = {\n    "
    tbl += cformat.multiline_fill(['{%i, 0x%04x}' % (k, idx[k]) for k in sorted(idx.keys())], ',', 4)
    tbl += "};\n"
    return tbl

def generate_GB_data_cpp():
    f = cformat.open_cpp_file_for_write('GB_18030_data')
    f.write(GB_double_byte_table())
    f.write("\nstd::vector<std::vector<codepoint_t>> & get_GB_DoubleByteTable() {\n   return GB_DoubleByteTable;\n}\n")
    f.write(GB_range_table())
    f.write("\nstd::vector<std::pair<unsigned, unsigned>> & get_GB_RangeTable() {\n   return GB_RangeTable;\n}\n")
    f.close()

if __name__ == "__main__":
    generate_GB_data_cpp()



