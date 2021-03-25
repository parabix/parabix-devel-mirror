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
       * Once the codeword for the phrase is calculated, we add the first word of the phrase to a fallBack data to check if it can be encoded as a smaller phrase (of 3,2 or 1 word).
       * The fallback mechanism continues until all the words of the longer phrase are compressed either in smaller group of words or individual words. Only the phrases/ words of length > 2 are compressed effectively as the maximum codeword length currently being handled is 2 bytes.
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
  1. For every byte in the word 'alpha', starting with 'a' (which has UTF-8 value 67), every bit of 'a' `(011100001)` is XOR'd with a bit of 'a' specified by the index position in bitmix[0] array. This is the first step of shuffling the bits of word. This is repeated for all the bytes of a phrase.
>
    Binary equivalent of 'alpha' = 01100001 01101100 01110000 01101000 01100001
    Bit shuffled 'alpha'         = 00010001 10111110 00100001 10111000 00010001

  2. Using the index positions from the next 4 bitmix arrays, we mix bits from word's prefix into the word's suffixes in the similar way mentioned above.
    
   * Referring to bitmix[1], starting from second byte 'l' till the last byte 'a', every bit of the byte is XOR'd with the bit from previous byte(2^0) specified by the index of bitmix[1].
>
     bit shuffled 'alpha'           = 00010001 10111011 01011111 11111001 01111101

  * Referring to bitmix[2], starting from third byte 'p' till the last byte 'a', every bit of the byte is XOR'd with the bit from two byte before current(2^1) byte specified by the index of bitmix[2].
>
    bit shuffled 'alpha'           = 00010001 10111011 01011001 01000111 00001010

  * Referring to bitmix[3], starting from fourth byte 'h' till the last byte 'a', every bit of the byte is XOR'd with the bit from four bytes before current(2^2) byte specified by the index of bitmix[3].
>
    bit shuffled 'alpha'           = 00010001 10111011 01011001 01000111 10000010

  * The first byte of this shuffled word acts as the suffix of codeword. i.e, 0x11 for 'alpha'.
> 
    Further experiments with fetching the byte corresponding to the length of the phrase as the suffix of codeword is also tried to utilize the entropy of the data.

* The prefix of the codeword is specific to the length of the word. The range of prefixes is as follows:

    | Length | start byte  | end byte | codeword space  |
    | -------|:-----------:|---------:|----------------:|
    |   3    |    0xC0     |   0xC3   |     1024        |
    |   4    |    0xC4     |   0xC8   |     1024        |
    |  5-8   |    0xC9     |   0xCF   |     2048        |
    |  8-16  |    0xD0     |   0xD7   |     2048        |
    |  17-32 |    0xD8     |   0xDF   |     2048        |

* For prefixes in the range C0-DF, we need to ensure that the suffix byte is not in the range 0x80 - 0xBF to avoid confusing with valid UTF-8 2-byte sequences. That does effectively reduce our codeword space.

>
    One of the experiments done to in order to extend the codeword space and overcome collissions, we increase the number of suffix bytes by reading the following byte from the shuffled bits.

* The prefix is calculated based on the current availability of unused bits in the prefix byte to be used.

### Prefix determination experiments:

  * Sub-divide the prefix range further based on number of bytes in the phrase.
  * Extended codeword prefix range: [0xC0, 0xC4, 0xC8, 0xD0, 0xF8]
  
    | Length | Prefix      |
    | -------|:-----------:|
    |   3    |  0xC0-0xC3  |
    |   4    |  0xC4-0xC7  |
    |   5    |  0xC8, 0xCC |
    |   6    |  0xC9, 0xCD |
    |   7    |  0xCA, 0xCE |
    |   8    |  0xCB, 0xCF |
    |   9    |  0xD0, 0xD8 |
    |  10    |  0xD1, 0xD9 |
    |  11    |  0xD2, 0xDA |
    |  12    |  0xD3, 0xDB |
    |  13    |  0xD4, 0xDC |
    |  14    |  0xD5, 0xDD |
    |  15    |  0xD6, 0xDE |
    |  16    |  0xD7, 0xDF |
    | 17, 25 |  0xF8       |
    | 18, 26 |  0xF9       |
    | 19, 27 |  0xFA       |
    | 20, 28 |  0xFB       |
    | 21, 29 |  0xFC       |
    | 22, 30 |  0xFD       |
    | 23, 31 |  0xFE       |
    | 24, 32 |  0xFF       |

### Compression 2.0 - length-group-phrase-compression

With the current compression scheme of fallback mechanism, we were creating a dependency among codewords. To decode the current codeword, we had to decode all the previous codewords and consider the corresponding phrases into consideration to determine the phrase corresponding to the current codeword being decompressed. An example of this would be explained in the decompression section below.

In order to overcome this dependency, we now identify all the phrases comprised of 4 bytes. Among those phrases, we analyze the repeated phrases and start calculating codewords following the stpes mentioned in the Hash calculation section. These repeated phrases are replaced by the appropriate codewords. This keeps repeating for every segment of 1MB. 

Once we have identified all the 4 word phrases (maximum number of words in phrases could be determined prior to execution) in a segment and encode them accordingly, this partially encoded segment is passed to identify the phrases containing 3 words and are compressed further. This step continues till the input boils down to compressing individual words or retaining them as plaintext in the compressed text. This technique essentially allows us to decode a codeword independently while decompression is being done.

### Detailed steps of the process are explained below with an example:

* For the specified maximum number of words in a phrase, we first analyze the input to identify phrases of specified length(working with 4 in the example) which are occurred the most and compress them in a segment. This is repeated for every segment of 1MB.
* This first level of compressed data is given next to identify plaintext sequences that could not benefit from phrase of 4 words compression. These sequences are further broken into phrases of 3 words and go through the process of identification of repeated phrases to compress them.
* The same sequence continues till the number of words in phrases is reduced down to 1 or plaintext entry of the word retains in the final compressed data.

>
    A phrase is comprised of words. Currently we are not treating empty space and new line character as a word in the phrase to better utilize the definition of a phrase.

We'll now have all the longer and shorter phrases in the compressed text in plaintext atleast once that helps in fast full-text search over the compressed data.

For the same example snippet text considered above, the compressed (codewords represented in their hexadecimal values) data is shown below:

\nThe modern English alphabet is a Latin`\xd8U` consisting of 26 letters, each having an upper- and lower-case form.\nIt originated around the 7th century from`\xc9\xca` script. Since then,`\xcb\xfa` have been added or removed to give`\xc7\x63` current Modern`\xf8\x6f``\xd5\x18` with no diacritics, diagraphs, and special characters. \n`\xc3\x17` word`\xd5\x82` compound of`\xc7\x63` first two`\xcb\xfa``\xca\x9a` Greek`\xd8\x55`, alpha and beta.

The dictionary generated for this snippet looks something like this:

codewordHashTableList table size

| phrase len |       phrase           |           codeword         |
| -----------|:----------------------:|---------------------------:|
|    3       |       'The'            |          \xc3\x17          |
|    4       |       ' the'           |          \xc7\x63          |
|    6       |      ' Latin'          |          \xc9\xca          |
|    7       |      ' of the'         |          \xca\x9a          |
|    8       |      ' letters'        |          \xcb\xfa          |
|    9       |      ' alphabet'       |          \xd8\x55          |
|   14       |    ' alphabet is a'    |          \xd5\x82          |
|   14       |    ' of 26 letters'    |          \xd5\x18          |

## Results

The compression results for some files with diverse input and bigger file sizes are as follows:

>

    1. Input file : UnicodeData.txt - 1851767 bytes
    
    sh-3.2# time python3 ztf-hash-tests.py -c UnicodeData.txt > output.debug 
    
    Compressed file: 
    output.length-group-phrase-compression.z - 633409 bytes

    Statistics:
    Total words: 444646 (only in the last segment calculated)
    collisions:  534531 phrases could have been compressed too if larger codeword space available.
    
    hash table size for phrases of length 3,4,5-8,9-16,17-32 respectively.
    96
    110
    512
    1020
    1522

    Looking at hash table size we can say that mostly phrases of length 5-8 were high in number which had scope of benefiting from compression.

    =============================================================================================
    2. Input file : viwikibooks-20141221-pages-articles.xml - 11846638 bytes

    sh-3.2# time python3 ztf-hash-tests.py -c viwikibooks-20141221-pages-articles.xml > output.debug 

    real	66m41.913s
    user	58m8.003s
    sys   8m11.748s
    
    sh-3.2# ls -l
    -rw-r--r--   1 root      staff   6258218 24 Mar 03:09 output.length-group-phrase-compression.z

    Statistics:
    Total words: 352332 (only in the last segment calculated)
    collisions:  5092070
    hash table size for phrases of length 3,4,5-8,9-16,17-32 respectively.
    128
    128
    512
    1024
    2038
    
    Total number of phrases identified of length 3,4,5-8,9-16,17-32 respectively.
    22589
    54465
    403692
    1200163
    765718

    =============================================================================================
    
    3. Input file: idwikibooks-20141221-pages-articles.xml - 15202378
    
    sh-3.2# time python3 ztf-hash-tests.py -c idwikibooks-20141221-pages-articles.xml > output.debug 
    real	118m29.600s
    user	100m20.786s
    sys	15m1.254s

    sh-3.2# ls -l
    total 300728
    -rw-r--r--   1 root      staff  10467660 24 Mar 11:02 output.length-group-phrase-compression.z

    Total words: 54898 (only in the last segment calculated)
    collisions: 6168905
    hash table size for phrases of length 3,4,5-8,9-16,17-32 respectively.
    128
    128
    512
    1024
    2048

    Total number of phrases identified of length 3,4,5-8,9-16,17-32 respectively.
    20164
    51122
    549614
    2037283
    3118146

    =============================================================================================

    * number of words in phrase increased to 6 from 4:
    4. Input file: idwikibooks-20141221-pages-articles.xml - 15202378
    sh-3.2# time python3 ztf-hash-tests.py -c idwikibooks-20141221-pages-articles.xml > output.debug 

    real	234m1.487s
    user	173m43.152s
    sys	26m53.563s
    
    sh-3.2# ls -l
    total 321032
    -rw-r--r--   1 root      staff  15202378 23 Mar 21:03 idwikibooks-20141221-pages-articles.xml
    -rw-r--r--   1 root      staff  10273346 24 Mar 15:04 output.length-group-phrase-compression.z

    Total words: 54898 (only in the last segment calculated)
    collisions: 7637386
    hash table size for phrases of length 3,4,5-8,9-16,17-32 respectively.
    128
    128
    512
    1024
    2048
  
    Total number of phrases identified of length 3,4,5-8,9-16,17-32 respectively.
    19714
    48900
    559503
    2617346
    5303420

* With the current available results and looking at the number of collisions, we can say that the potential of compression is higher if we have a larger codeword space.

## Decompression:

The basic flow of decompression follows the steps below:

1. While parsing the compressed byte sequence, if any valid/invalid start byte of UTF-8 sequence is observed(0xC0-0xFF) with invlaid suffix byte, we capture 2 consecutive bytes and treat it as a codeword.
2. The codeword corresponds to one of the phrases occurred previously in the compressed byte sequence.
3. We can identify the length range of decompressed phrase from the codeword prefix.

    | start byte  | end byte | Length |
    |:-----------:|---------:|--------|
    |    0xC0     |   0xC3   |   3    |
    |    0xC4     |   0xC8   |   4    |
    |    0xC9     |   0xCF   |  5-8   |
    |    0xD0     |   0xDF   |  8-16  |
    |    0xF8     |   0xFF   |  17-32 |

4. Upon finding the length range of phrase from codeword prefix, we use rolling hash method to find the actual phrase that corresponds to the codeword.

Example snippet:

`The modern English alphabet is a Latin alphabet consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from Latin script. Since then, letters have been added or removed to give the current Modern English alphabet of 26 letters with no diacritics, diagraphs, and special characters. 
The word alphabet is a compound of the first two letters of the Greek alphabet, alpha and beta.`

Compressed form:
For ease of reading, the codewords are represented in their hexadecimal format wrapped as a bytearray (b'bytes')

The modern English alphabet is a Latin `\xc8\xcc` consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from `\xc9\xbb` script. Since then, `\xc8\x42` have been added or removed to give `\xc1\x21` current Modern`\xd9\xa9``\xd1\xc9` with no diacritics, diagraphs, and special characters. 
`\xc1\x2d` word `\xd1\x21` compound of `\xc1\x21` first two `\xc8\x42``\xc8\xe8` Greek `\xc8\xcc`, alpha and beta.

The hashtable of compressed phrases is as below:

### Phrases of lenght 3:

('the', [bytearray(b'\xc1\x21')])

('The', [bytearray(b'\xc1-')])

### Phrases of length 4:

### Phrases of length 5-8:

('alphabet', [bytearray(b'\xc8\xcc')])

('Latin', [bytearray(b'\xc9\xbb')])

('letters', [bytearray(b'\xc8B')])

(' of the', [bytearray(b'\xc8\xe8')])

### Phrases of length 9-16:
(' of 26 letters', [bytearray(b'\xd1\xc9')])
(' alphabet is a', [bytearray(b'\xd1-')])

### Phrases of length 17-32:
(' English alphabet', [bytearray(b'\xd9\xa9')])

5. What rolling hash does here -

* When a codeword is encountered, it looks at the previous plaintext bytes in an incremental manner to find the first matching phrase that corresponds to the codeword observed.
* For codeword '\xc8\xcc', from the prefix we know that the decompressed phrase is of length 5-8.
* From the plaintext retreived so far, we segment all the phrases of length 5, 6, 7 and 8 and calculate their codeword until we find the phrase which matches the current codeword.

PROBLEM: As the compression is achieved by analyzing the input data to figure out most occurring phrases, while decompressing and finding the matching phrase, if some other phrase also generates the same codeword before we find the actual phrase that corresponds to the codeword at hand, we might end up replacing the codeword with incorrect phrase. We can see that in the example decompression of above input snippet.

The modern English alphabet is a Latin alphabet consisting of 26 letters, each having an upper- and lower-case form.
It originated around the 7th century from Latin script. Since then, letters have been added or removed to give the current Modern modern English alphabet`The modern` with no diacritics, diagraphs, and special characters. 
The word`modern English` compound of the first of the Greek alphabet, alpha and beta.

Hashtable generated while decompression:

### Phrases of length 3:
b'\xc1!', 'the'

b'\xc1-', 'The'

### Phrases of length 4:

### Phrases of length 5-8:

b'\xc8\xcc', 'alphabet'

b'\xc9\xbb', 'Latin'

b'\xc8B', 'letters'

b'\xc8\xe8', '.\nIt '

### Phrases of length 9-16:

b'\xd1\xc9', 'The modern'

b'\xd1-', 'modern English'

### Phrases of length 17-32:

b'\xd9\xa9', ' modern English alphabet'

Experiments run to tackle the problem:

* Tried to perform bitmix of every previous byte with the subsequent byte of the phrase while generating the codeword prefix and fetch the byte at position corresponding to length of phrase to maintain the randomness of the data.

* The problem at hand is solved for this small example when we pre-determine the length of the phrase based on the codeword prefix. However, this is not very helpful in the decompression of larger text data.
