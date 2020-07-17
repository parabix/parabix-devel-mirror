#
# Parse a WHATWG specification an encoding index file and return
# a dictionary mapping "pointers" to code points.
#
def parse_WHATWG_index_file(enc_name):
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
