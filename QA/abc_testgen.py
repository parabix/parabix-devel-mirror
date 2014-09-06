#
# abc_testgen.py
#
# Generate grep test cases for ab*c pattern to segment
# handling support (long lines, b* crossing segment boundaries).
#
from random import randint
import os
import shutil
#
#
output_home = 'abc'

#
# Generate a line that matches ab*c with a given offset from 
# the line beginning, a given length of the match and a given
# total line length (including new line).
#
def generate_line_to_match(offset, match_length, line_length):
  return "%sa%sc%s\n" % (('_' * offset), ("b" * (match_length - 2)), ('_' * (line_length - (offset + match_length) - 1)))

#
# Generate lines that do not match ab*c of total length lgth.
#
def generate_non_match_lines(lgth):
  lgth_remain = lgth
  linedata = ""
  while lgth_remain > 0:
    this_line_lgth = randint(1, lgth_remain)
    if this_line_lgth == 1: linedata += "\n"
    else: linedata += "a%s\n" % ("b" * (this_line_lgth - 2))
    lgth_remain -= this_line_lgth
  return linedata

def generate_case(line_pos, match_pos, match_length, line_length, extra):
  file_name = "ab{%i}c@%i.txt" % (match_length - 2, line_pos+match_pos)
  line_to_match = generate_line_to_match(match_pos-line_pos, match_length, line_length)
  f1 = file(output_home + "/ExpectedOutput/" + file_name, "w")
  f1.write(line_to_match)
  f1.close()
  f2 = file(output_home + "/TestFiles/" + file_name, "w")
  f2.write(generate_non_match_lines(line_pos))
  f2.write(line_to_match)
  f2.write(generate_non_match_lines(extra))
  f2.close()

def main():
  if os.path.exists(output_home + ".bak" ):
    shutil.rmtree(output_home + ".bak")
  if os.path.exists( output_home ):
    shutil.move(output_home, output_home + ".bak")
  os.mkdir(output_home)
  os.mkdir(output_home + "/TestFiles/")
  os.mkdir(output_home + "/ExpectedOutput/")

  for block_size in [64, 128, 1024]:
     for segment_blocks in [2,3,5,11,15,19,25,30,31,32,43]:
        segment_base = block_size * segment_blocks
        for segments_in_match in range(4):
          additional_blocks = randint(0, segment_blocks - 1)
          match_blocks = segments_in_match * segment_blocks + additional_blocks
          match_extra = randint(0, block_size - 1)
          match_length = match_blocks * block_size + match_extra
          min_segment_offset = segment_base - (additional_blocks * block_size + match_extra)   
          max_segment_offset = segment_base - match_extra
          match_abs_pos = randint(min_segment_offset, max_segment_offset) + randint(0, 10) * segment_base
          line_abs_pos = match_abs_pos - randint(1, match_abs_pos)
          line_length = (match_abs_pos - line_abs_pos) + match_length + randint(1, segment_base)
          extra = randint(0, segment_base * 4)
          generate_case(line_abs_pos, match_abs_pos, match_length, line_length, extra)

if __name__ == "__main__":
  main()


 
