import codecs
import binascii
# https://medium.com/analytics-vidhya/python-dictionaries-are-ordered-now-but-how-and-why-5d5a40ee327f
# dict in Python 3.7 is ordered by default, passing the use of OrderedDict for now
from collections import OrderedDict


class Compressor:
    def __init__(self):
        self.name = "edelta"
        self.hashTable = dict()
        self.encodePrefix = 192  # b'\xC0'
        self.encodeSuffix = 0    # b'\x00'
        self.compressed = bytearray(b'')
        self.expandSym = 218   # b'\xDA'
        self.words = []
        self.expandWord = ""

    def Name(self):
        return 'words'

    def expandSymbol(self, prevPos, currIndex, hashedWordsLen):
        expandWord = ""
        maxLen = len(self.words)
        while(currIndex < maxLen and
              self.words[prevPos] == self.words[currIndex]):
            expandWord += self.words[currIndex]
            currIndex += 1
            prevPos += 1
        return expandWord, currIndex

    # for every occurrence of a symbol which is already encoded, try to find the longest match
    # of the sequence of symbols which are already encoded and encode the complete sequence with
    # a single encoded unit. This even helps encoding symbols of RLE format.
    # TODO: we can restrict the lookahead range for any symbol for faster compression.
    def Compress(self, words):
        self.words = words
        index = 0

        while index < (len(words)-1):
            word = words[index]
            index += 1
            expandedWord = ""
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            hash_pos = self.hashTable.get(word, None)
            if hash_pos:
                hashVal = hash_pos[0]
                posArray = hash_pos[1]
                keys = []
                keys = list(self.hashTable.keys())
                for pos in posArray:
                    expandedWord, newIndex = self.expandSymbol(
                        pos, index, len(keys))
                    if len(expandedWord) > len(self.expandWord):
                        self.expandWord = expandedWord
                index = newIndex

                extendedHash = bytearray(b'')
                if len(self.expandWord) > len(word):
                    self.hashTable[word][1].append(index)
                    #print(self.expandWord, 'self.expandWord')
                    extendedHash.append(self.expandSym)
                    extendedHash.append(len(self.expandWord))
                if len(word) == 1:
                    self.compressed += bytearray(word, 'utf-8')
                else:
                    self.compressed += hashVal
                if len(word) > 1 and extendedHash:
                    self.compressed += extendedHash
            else:
                self.encodeSuffix += 1
                if self.encodeSuffix == 255:
                    self.encodePrefix += 1
                    self.encodeSuffix = 0
                encodedWord = bytearray(b'')
                encodedWord.append(self.encodePrefix)
                encodedWord.append(self.encodeSuffix)
                self.hashTable[word] = [encodedWord, [index]]
                self.compressed += bytearray(word, 'utf-8')
        return self.compressed
    # Next steps:
    # 1.
    #  lookahead of arbitrary position in the input text is not appropriate for high-performance compression
    # For every symbol A which is already encoded, if we can locate the positions of occurrences of A within
    # certain range (next 256 symbols) and expand them simultaneously to find the longest common symbol
    # sequence that could be compressed as a single encoded unit might be useful!

    # 2.
    # Extending CAFTS algorithm for symbol/word based compression
    # Ref - https://www.researchgate.net/publication/304492864_A_new_compression_algorithm_for_fast_text_search
    # for every pair of words, consider next 1,2 or 3 words and
    # look for those set of words in the sub-dictionary of pair of words encoded.
    # For eg: # Disable VCS-based implicit rules.
    # word list: '#', ' ', 'Disable', ' ', 'VCS', '-', 'based', ' ', 'implicit', ' ', 'rules', '.'
    # starting with pair "# ", if "#" and " " are found in hashTable in consecutive positions/ encoded together
    # check if the next 3 keys in the hashTable are same as  "Disable VCS-"
    # if yes, encode the 3 words with the position offset of the 3 words WRT position of "# " (common prefix,  sum of suffixes % len(3 words))
    # else, fall back to the individual encoding of the words.
    # upon group encoding, move the cursor to select next pair of words as " Disable" and continue the process
