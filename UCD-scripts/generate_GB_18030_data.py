import cformat
import WHATWG_parser

def GB_double_byte_table():
    idx = WHATWG_parser.parse_WHATWG_index_file('gb18030')
    tbl = "std::vector<std::vector<UCD::codepoint_t>> GB_DoubleByteTable = {\n"
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

def GB_double_byte_table():
    idx = WHATWG_parser.parse_WHATWG_index_file('gb18030')
    tbl = "std::vector<unsigned> GB_DoubleByteTable = {\n    "
    tbl += cformat.multiline_fill(['0x%04x' % idx[k] for k in sorted(idx.keys())], ',', 4)
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
    f.write("\nstd::vector<UCD::codepoint_t> & get_GB_DoubleByteTable() {\n   return GB_DoubleByteTable;\n}\n")
    f.write(GB_range_table())
    f.write("\nstd::vector<std::pair<unsigned, unsigned>> & get_GB_RangeTable() {\n   return GB_RangeTable;\n}\n")
    f.close()
    f = cformat.open_header_file_for_write('GB_18030_data')
    cformat.write_imports(f, ['<vector>', '<UCD/unicode_set.h>'])
    f.write("\nstd::vector<UCD::codepoint_t> & get_GB_DoubleByteTable();\n")
    f.write("\nstd::vector<std::pair<unsigned, unsigned>> & get_GB_RangeTable();\n")
    cformat.close_header_file(f)

if __name__ == "__main__":
    generate_GB_data_cpp()
