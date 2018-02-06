
# ------------
# Block Format:
# Block Size: 4byte (Little-endian)
# data: size determined by the Value Block Size (does not include the first bit)
# (Block Checksum) 0~4 bytes

def decode(file, block_start, has_block_checksum, has_content_checksum):
    ret_block_info = []

    current_position = block_start

    end_pos = len(file)
    end_pos -= 4  # End Mark
    if has_content_checksum:
        end_pos -= 4

    while current_position < end_pos:
        block_size = 0
        for i in range(4):
            block_size = block_size + (file[current_position + i] << (i * 8))

        real_block_size = block_size & 0x7fffffff
        highest_bit = (block_size >> 31) & 1
        is_compressed = not highest_bit

        current_position += 4

        block_start = current_position
        current_position += real_block_size
        block_end = current_position

        if has_block_checksum:
            current_position += 4

        ret_block_info.append({
            "block_start": block_start,
            "block_end": block_end,
            "is_compressed": is_compressed
        })

    return ret_block_info