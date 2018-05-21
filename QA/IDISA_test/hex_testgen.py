#
# line_testgen.py
#
# Generate test case for Unicode line length tests (.{m} grep patterns).
#
from random import randint
import codecs
#
def generate_hex_of_length(n):
  line = ""
  for i in range(n):
    hval = randint(0, 15)
    if (hval < 10): cp = ord('0') + hval
    else: cp = ord('a') + hval - 10
    line += chr(cp)
  return line

def main():
  stream_lgth = 1<<16
  f1 = codecs.open("randhex%ia" % stream_lgth, mode='w', encoding='utf-8')
  f1.write(generate_hex_of_length(stream_lgth))
  f1.close()
  f2 = codecs.open("randhex%ib" % stream_lgth, mode='w', encoding='utf-8')
  f2.write(generate_hex_of_length(stream_lgth))
  f2.close()

if __name__ == "__main__":
  main()
