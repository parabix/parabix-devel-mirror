import decoder.FrameDecoder as FrameDecoder
import decoder.BlockDecoder as BlockDecoder
import decoder.SequenceDecoder as SequenceDecoder

class LZ4Decoder:
    def decode(self, input_file, output_file, options):
        with open(input_file, "rb") as f:
            file_content = f.read()

        frame_info = FrameDecoder.decode(file_content)
        block_info = BlockDecoder.decode(file_content, frame_info['block_start'], frame_info['has_block_checksum'], frame_info['has_content_checksum'])
        result = SequenceDecoder.decode(file_content, block_info, options)
        with open(output_file, 'wb') as f:
            f.write(result)



