
# ---------------
# Frame Format:
# Magic Number -- 4 bytes
# Frame Descriptor -- 3-11 bytes
# Data Blocks
# EndMark  -- 4 bytes
# C. Checksum -- 0-4 bytes



minFilesize = (
    4 +         # Magic number
    3 +         # Frame descriptor (3-11 bytes)
    4          # End mark
)

# Little-endian.
def decode(file):
    result = {}
    __check_magic_number(file)
    return __decodeFrameDescriptor(file)

MagicNumber = 0x184D2204


def __check_magic_number(file):
    res = 0
    for i in range(4):
        res = (res << 8) + file[3 - i]  # Little-endian.
    if res != MagicNumber:
        raise Exception('Invalid LZ4 File')


def __decodeFrameDescriptor(file):

    descriptor_offset_base = 4
    FLG_byte = file[descriptor_offset_base]
    has_content_checksum = (FLG_byte >> 2) & 1
    has_content_size = (FLG_byte >> 3) & 1
    has_block_checksum = (FLG_byte >> 4) & 1

    version_number = FLG_byte >> 6
    if version_number != 1:
        raise Exception('Unsupport Version Number')
    if has_content_size:
        block_start = 15
    else:
        block_start = 7

    return {
        "has_content_checksum": has_content_checksum,
        "has_content_size": has_content_size,
        "has_block_checksum": has_block_checksum,
        "block_start": block_start
    }
