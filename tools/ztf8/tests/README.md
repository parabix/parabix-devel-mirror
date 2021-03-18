# Phrase based ztf-hash 

### Step 1: Unicode word segmentation of the input data
* The input text is segmented into a list of individual words/symbols in accordance with the Unicode word segmentation algorithm. (https://unicode.org/reports/tr29/)

  * **Example:** Input text:
The modern English alphabet is a Latin alphabet consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from Latin script. Since then, letters have been added or removed to give the current Modern English alphabet of 26 letters with no diacritics, diagraphs, and special characters. 
The word alphabet is a compound of the first two letters of the Greek alphabet, alpha and beta.

  * Number of words (N) - 155 
  * Unicode words - ['The', ' ', 'modern', ' ', 'English', ' ', 'alphabet', ' ', 'is', ' ', 'a', ' ', 'Latin', ' ', 'alphabet', ' ', 'consisting', ' ', 'of', ' ', '26', ' ', 'letters', ',', ' ', 'each', ' ', 'having', ' ', 'an', ' ', 'upper', '-', ' ', 'and', ' ', 'lower', '-', 'case', ' ', 'form', '.', '\n', 'It', ' ', 'originated', ' ', 'around', ' ', 'the', ' ', '7th', ' ', 'century', ' ', 'from', ' ', 'Latin', ' ', 'script', '.', ' ', 'Since', ' ', 'then', ',', ' ', 'letters', ' ', 'have', ' ', 'been', ' ', 'added', ' ', 'or', ' ', 'removed', ' ', 'to', ' ', 'give', ' ', 'the', ' ', 'current', ' ', 'modern', ' ', 'English', ' ', 'alphabet', ' ', 'of', ' ', '26', ' ', 'letters', ' ', 'with', ' ', 'no', ' ', 'diacritics', ',', ' ', 'diagraphs', ',', ' ', 'and', ' ', 'special', ' ', 'characters', '.', ' ', '\n', 'The', ' ', 'word', ' ', 'alphabet', ' ', 'is', ' ', 'a', ' ', 'compound', ' ', 'of', ' ', 'the', ' ', 'first', ' ', 'two', ' ', 'letters', ' ', 'of', ' ', 'the', ' ', 'Greek', ' ', 'alphabet', ',', ' ', 'alpha', ' ', 'and', ' ', 'beta', '.', '\n']

* Based on the maximum number of words to be considered in a phrase while compression, the input text is split into appropriate phrases. We are considering maximum 4 words in a phrase with a flexibility to use phrases of length < 4 as well. 

* To start with, the phrases look like this. For input of N words, there are N-3 phrases of length 4. The example snippet consists only of those phrases which have shown the advantage of phrase compression for the example considered.

`('The', ' ', 'modern', ' ')`
`(' ', 'modern', ' ', 'English')`
`('modern', ' ', 'English', ' ')`
`(' ', 'English', ' ', 'alphabet')`
`('English', ' ', 'alphabet', ' ')`
`<SNIP>`
`(' ', 'modern', ' ', 'English')`
`('modern', ' ', 'English', ' ')`
`(' ', 'English', ' ', 'alphabet')`
`('English', ' ', 'alphabet', ' ')`
`<SNIP>`

### Step 2: Check for longest phrase to be encoded in a decreasing order of number of words in a phrase

* Once all the word phrases of 4 words are identified, start checking each phrase to have already occurred in the text parsed and encoded so far. We'll have 2 cases to handle:
    1. If the phrase was already seen and has an entry for corresponding codeword in the hash table: if this is the case, we would directly replace the phrase with a codeword. The steps of calculating codeword of a word is inspired by BixHash kernel of ztf-hash application and will be explained in a separate section.

    #create an actual dictionary specific for this small snippet of text that actually shows some compression of repeated phrases

    | Phrase                  | Codeword      | # words | length |
    | ------------------------|:-------------:|--------:|-------:|
    | ' modern English'       | 0xD0 0xF6     |    4    |   15   |
    | ' alphabet is'          | 0xD1 0xA9     |    4    |   12   |
    | ' of the'               | 0xC9 0xA9     |    4    |   7    |
    | ' alphabet '            | 0xD0 0xF6     |    3    |   10   |
    | ' Latin '               | 0xC8 0x3C     |    3    |   7    |
    | 'The '                  | 0xC4 0x8C     |    2    |   4    |
    | 'letters'               | 0xC9 0xC9     |    1    |   7    |

    2. If the phrase is seen for the first time in the text, we memoize the occurence and calculate the corresponding codeword value for that phrase only if it is occurring repeatedly. The calculated codeword is saved in one of the hash tables based on the length of the phrase. The memoization of phrase is carried out to carefully utilize the codeword space and avoid unnecessary collisions.
       * Once the codeword for the phrase is calculated, we add the first word of the phrase to a fallBack array to check if it can be encoded as a smaller phrase (of 3,2 or 1 word).
       * The fallback mechanism continues until all the words of the longer phrase are compressed either in smaller group of words or individual words. Only the phrases/ words of length > 2 are compressed as the maximum codeword length currently being handled is 4 bytes.
    > The hash tables are separate for phrases of different length. If the codeword for two different phrases of different length is same, in order to utilize the opportunity of compressing as many  phrases as possible, we keep the phrases (key) and their codewords (values) in different hash tables.

### Hash calculation:

* The calculation of codeword of a phrase goes through a process of bitwirse operations that exploits the ordering of the byte sequences.
* We make use of randomly generated arrays(bitMix arrays) to help with bit shuffling of every byte of the word.

### Bitmix arrays
>
          [[4, 1, 7, 2, 0, 6, 5, 3],
           [1, 2, 4, 5, 0, 3, 6, 7],
           [0, 5, 4, 6, 2, 3, 7, 1],
           [7, 1, 6, 4, 3, 0, 5, 2],
           [3, 2, 5, 4, 7, 6, 0, 1]]

Let's walk through the steps of codewrod calculation for word 'alpha'
  1. For every byte in the word 'alpha', starting with 'a' (which has UTF-8 value 67), every bit of 'a' `(011100001)` is XOR'd with a bit of 'a' specified by the index position in bitmix[0] array. This is the first step of shuffling the bits of word.
    * `Binary equivalent of 'alpha' = 01100001 01101100 01110000 01101000 01100001`
    * `Bit shuffled 'alpha'         = 00010001 10111110 00100001 10111000 00010001`

  2. Using the index positions from the next 4 bitmix arrays, we mix bits from word's prefix into the word's suffixes in the similar way mentioned above.
    * Referring to bitmix[1], starting from second byte 'l' till the last byte 'a', every bit of the byte is XOR'd with the bit from previous byte(2^0) specified by the index of bitmix[1].
    `bit shuffled 'alpha'           = 00010001 10111011 01011111 11111001 01111101`

    * Referring to bitmix[2], starting from third byte 'p' till the last byte 'a', every bit of the byte is XOR'd with the bit from two byte before current(2^1) byte specified by the index of bitmix[2].
    `bit shuffled 'alpha'           = 00010001 10111011 01011001 01000111 00001010`

    * Referring to bitmix[3], starting from fourth byte 'h' till the last byte 'a', every bit of the byte is XOR'd with the bit from four bytes before current(2^2) byte specified by the index of bitmix[3].
    `bit shuffled 'alpha'           = 00010001 10111011 01011001 01000111 10000010`

    * The first byte of this shuffled word acts as the suffix of codeword. i.e, 0x11 for 'alpha'

* The prefix of the codeword is specific to the length of the word. The range of prefixes is as follows:

    | Length | start byte  | end byte | codeword space  |
    | -------|:-----------:|---------:|----------------:|
    |   3    |    0xC0     |   0xC3   |     1024        |
    |   4    |    0xC4     |   0xC8   |     1024        |
    |  5-8   |    0xC9     |   0xCF   |     2048        |
    |  8-16  |    0xD0     |   0xD7   |     2048        |
    |  17-32 |    0xD8     |   0xDF   |     2048        |

    * In order to extend the codeword space and overcome collissions, we increase the number of suffix bytes by reading the following byte from the shuffled bits.

* The prefix is calculated based on the current availability of unused bits in the prefix byte to be used.

## Results: (To be filled)




## Decompression:

1. While parsing the compressed byte sequence, if any invalid start byte of UTF-8 sequence is observed(0xC0-0xDF), we capture 2 consecutive bytes and treat it as a codeword.
2. The codeword corresponds to one of the phrases occurred previously in the compressed byte sequence.
3. We can identify the length range of decompressed phrase from the codeword prefix.

    | start byte  | end byte | Length |
    |:-----------:|---------:|--------|
    |    0xC0     |   0xC3   |   3    |
    |    0xC4     |   0xC8   |   4    |
    |    0xC9     |   0xCF   |  5-8   |
    |    0xD0     |   0xD7   |  8-16  |
    |    0xD8     |   0xDF   |  17-32 |

4. Upon finding the length range of phrase from codeword prefix, we use rolling hash method to find the actual phrase that corresponds to the codeword.

The modern English ALPHABET is a Latin alphabet consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from Latin script. Since then, letters have been added or removed to give the current Modern English alphabet of 26 letters with no diacritics, diagraphs, and special characters. 
The word alphabet is a compound of the first two letters of the Greek alphabet, alpha and beta.

Compressed form:
For ease of reading, the codewords are represented in their hexadecimal format wrapped as a bytearray (b'bytes')

The modern English alphabet is a Latin (b'\xc8\xcc') consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from (b'\xc9\xbb') script. Since then, (b'\xc8B') have been added or removed to give (b'\xc1!') current Modern(b'\xd9\xa9')(b'\xd1\xc9') with no diacritics, diagraphs, and special characters. 
(b'\xc1-') word (b'\xd1-') compound of (b'\xc1!') first two (b'\xc8B')(b'\xc8\xe8') Greek (b'\xc8\xcc'), alpha and beta.

The hashtable of compressed phrases is as below:

Phrases of lenght 3:
('the', [bytearray(b'\xc1!'), 84])
('The', [bytearray(b'\xc1-'), 20])

Phrases of length 4:

Phrases of length 5-8:
('alphabet', [bytearray(b'\xc8\xcc'), 15])
('Latin', [bytearray(b'\xc9\xbb'), 58])
('letters', [bytearray(b'\xc8B'), 68])
(' of the', [bytearray(b'\xc8\xe8'), 12])

Phrases of length 9-16:
(' of 26 letters', [bytearray(b'\xd1\xc9'), 92])
(' alphabet is a', [bytearray(b'\xd1-'), 120])

Phrases of length 17-32:
(' English alphabet', [bytearray(b'\xd9\xa9'), 88])

4.1 What rolling hash does here is -
* When a codeword is encountered, it looks at the previous plaintext bytes in an incremental manner to find the first matching phrase that corresponds to the codeword observed. 
* For codeword '\xc8\xcc', from the prefix we know that the decompressed phrase is of length 5-8.
* From the plaintext retreived so far, we segment all the phrases of length 5, 6, 7 and 8 and calculate their codeword until we find the phrase which matches the current codeword.
PROBLEM: As the compression is achieved by analyzing the input data to figure out most occurring phrases, while decompressing and finding the matching phrase, if some other phrase also generates the same codeword before we find the actual phrase that corresponds to the codeword at hand, we might end up replacing the codeword with incorrect phrase. We can see that in the example decompression of above input snippet.

The modern English alphabet is a Latin alphabet consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from Latin script. Since then, letters have been added or removed to give the current Modern modern English alphabetThe modern with no diacritics, diagraphs, and special characters. 
The wordmodern English compound of the first two letters.
It  Greek alphabet

Hashtable generated while decompression:

Phrases of length 3:
b'\xc1!', 'the'
b'\xc1-', 'The'

Phrases of length 4:

Phrases of length 5-8:
b'\xc8\xcc', 'alphabet'
b'\xc9\xbb', 'Latin'
b'\xc8B', 'letters'
b'\xc8\xe8', '.\nIt '

Phrases of length 9-16:
b'\xd1\xc9', 'The modern'
b'\xd1-', 'modern English'

Phrases of length 17-32:
b'\xd9\xa9', ' modern English alphabet'

* To tackle this, we tried to perform bitmix of every previous byte with the subsequent byte of the phrase while generating the codeword prefix and fetch the byte at position corresponding to length of phrase to maintain the randomness of the data.
