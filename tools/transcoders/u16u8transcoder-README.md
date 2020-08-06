## u32u8 Transcoder Using Parallel Bit Streams

## Example

As a simple running example, we use the following UTF-32 codepoints.
```
𐀣 ॐ ā c
```

The corresponding UTF16-BE codepoints with BOM in 16 parallel bitstream are represented as follows:

```
1   0   1   0   0   0   1   0   1
1   1   0   1   0   1   1   0   1
0   0   0   0   0   0   0   0   1
0   0   0   0   0   0   0   0   1
0   0   0   0   1   0   0   0   1
1   0   0   0   0   0   1   0   1
1   0   0   0   1   0   0   0   1
0   0   0   0   0   0   0   0   1
0   0   1   0   1   0   0   0   0   
0   0   0   0   0   0   0   0   1
0   0   0   0   0   0   1   0   1
0   0   0   0   1   0   1   1   1   
0   0   0   0   0   0   1   1   1   
0   0   0   0   0   0   0   0   1
0   0   0   0   0   0   1   1   1
0   0   0   0   0   0   1   1   1
```

The transcoded parallel bitstream of UTF8 basis bits represent one or more bit positions of UTF8 codeunit sequences.

basis[0] represents bits 7, 15, 23, 31 of 4-byte UTF8 sequence, bits 7, 15, 23 of 3-byte UTF8 sequences, bits 7, 15 of 2-byte UTF8 sequences and bit 7 of ASCII codeunit.
Below tabular column represents the summary of bit positions of single/multibyte UTF8 sequences in 8-bit parallel bitstream.


| basis stream   | UTF-8 sequence length     | Bit position   |
| ---------------|:-------------------------:| --------------:|
| basis[0]       | 1-byte                    |  7             |
|                | 2-byte                    |  7, 15         |
|                | 3-byte                    |  7, 15, 23     |
|                | 4-byte                    |  7, 15, 23, 31 |
| basis[1]       | 1-byte                    |  6             |
|                | 2-byte                    |  6, 14         |
|                | 3-byte                    |  6, 14, 22     |
|                | 4-byte                    |  6, 14, 22, 30 |
| basis[2]       | 1-byte                    |  5             |
|                | 2-byte                    |  5, 13         |
|                | 3-byte                    |  5, 13, 21     |
|                | 4-byte                    |  5, 13, 21, 29 |
| basis[3]       | 1-byte                    |  4             |
|                | 2-byte                    |  4, 12         |
|                | 3-byte                    |  4, 12, 20     |
|                | 4-byte                    |  4, 12, 20, 28 |
| basis[4]       | 1-byte                    |  3             |
|                | 2-byte                    |  3, 11         |
|                | 3-byte                    |  3, 11, 19     |
|                | 4-byte                    |  3, 11, 19, 27 |
| basis[5]       | 1-byte                    |  2             |
|                | 2-byte                    |  2, 10         |
|                | 3-byte                    |  2, 10, 18     |
|                | 4-byte                    |  2, 10, 18, 26 |
| basis[6]       | 1-byte                    |  1             |
|                | 2-byte                    |  1, 9          |
|                | 3-byte                    |  1, 9, 17      |
|                | 4-byte                    |  1, 9, 17, 25  |
| basis[7]       | 1-byte                    |  0             |
|                | 2-byte                    |  0, 8          |
|                | 3-byte                    |  0, 8, 16      |
|                | 4-byte                    |  0, 8, 16, 24  |

The 16 parallel bitstreams are spreaded in accordance with the below deposit masks which helps in assembly of transcoded UTF8 codeunits.

```
dep0_5   = 1110 1100 1110 0100
dep6_11  = 1101 1010 1011 0010
dep12_15 = 1101 1001 1001 1001
```

1. dep0_5 indicates position of bits 2-7 of ASCII codepoints, bits 10-15 of 2-byte codepoints, bits 18-23 of 3-byte codeunits and bits 12-19 (except bits 16,17) 26-31 of 4-byte sequences.
2. dep6_11 indicates position of bits 2-7 of 2-byte sequences, bits 10-15 of 3-byte sequences and bits 4-7, 10-11, 20-23 of 4-byte sequences.
3. dep12_15 indicates the position of bits 4-7 of 3-byte sequences and bits 0-3 of 4-byte sequences.

The 16 parallel bitstreams are spreaded as follows in accordance to the deposit masks.

```
dep0_5   = 1110 1100 1110 0100  (offset=0)
           1.1. .... .1.. .1..
           11.. 1... 11.. .1..
           .... .... .... .1..
           .... .... .... .1..
           .... .1.. .... .1..
           1... .... .1.. .1..
```

```
dep6_11  = 1101 1010 1011 0010  (offset=6)
           1... ..1. .... ..1.
           .... .... .... ..1.
           ...1 ..1. .... ....
           .... .... .... ..1.
           .... .... ..1. ..1.
           .... ..1. ..11 ..1.
```

```
dep12+15 = 1101 1001 1001 1001  (offset=12)
           .... .... ...1 1..1
           .... .... .... ...1
           .... .... ...1 1..1
           .... .... ...1 1..1
```

UTF-8 Assembly

We make use of a few masks in order to effectively and accurately assemble UTF-8 code units which correspond to the UTF-16 codepoints.

1. dep0_5[0], dep6_11[0], dep12_15[0] , dep0_5[2] and dep6_11[2] are needed to calculate u8basis[0]. dep0_5[2] and dep6_11[2] are needed to extract bits 7 and 15 of 4-byte sequences.
(Need to optimize operations for fetching bits 7 and 15 of 4-byte sequences)

```
dep0_5[2] OR dep6_11[2] = 0001 0010 0000 0110       ..... (1)
advance(sur_sfx, 1)     = 0000 0000 0011 0000       ..... (2)
(1) AND (2)             = 0000 0000 0000 0000       ..... (3)
(3) OR ~(2)             = 1111 1111 1100 1111       ..... (4)
dep0_5[0] OR dep6_11[0]
OR dep12_15[0]          = 1010 0010 0101 1111       ..... (5)
(4) AND (5)             = 1010 0010 0100 1111       ..... (6)
~sur_pfx                = 1111 1111 1110 0111       ..... (7)
(6) AND (7)             = 1010 0010 0100 0111       ..... u8basis[0]
```

2. dep0_5[1], dep6_11[1], dep12_15[1] , dep0_5[3] and dep6_11[3] are needed to calculate u8basis[1]. dep0_5[3] and dep6_11[3] are needed to extract bits 6 and 14 of 4-byte sequences.

```
dep0_5[1] OR dep6_11[1]
OR dep12_15[1]          = 1100 1000 1100 0111
u8basis[1]              = 1100 1000 1100 0111       ..... u8basis[1]
```
We would need to follow operations similar to steps (1) through (4) in order to fetch bits 6 and 14 of 4-byte sequences.

3. dep0_5[2], dep6_11[2], dep12_15[2] , dep0_5[4] and dep6_11[4] are needed to calculate u8basis[2]. dep0_5[4] and dep6_11[4] are needed to extract bits 5 and 13 of 4-byte sequences.

```
dep0_5[2] OR dep6_11[2]
OR dep12_15[2]          = 0001 0010 0001 1111       ..... (8)
dep0_5[4] OR dep6_11[4] = 0000 0100 0010 0110       ..... (9)
sur_pfx                 = 0000 0000 0001 1000       ..... (10)
(9) AND (10)            = 0000 0000 0000 0000       ..... (11)
~(10)                   = 1111 1111 1110 0111       ..... (12)
(11) OR (12)            = 1111 1111 1110 0111       ..... (13)
(8) and (13)            = 0001 0010 0000 0111       ..... u8basis[2]
```

4. dep0_5[3], dep6_11[3], dep12_15[3] , dep0_5[5] and dep6_11[5] are needed to calculate u8basis[3]. dep0_5[5] and dep6_11[5] are needed to extract bits 4 and 12 of 4-byte sequences.
Bit 4 extraction could be achieved using alternate operations because 4th highest bit of 4-byte sequences is always 0.
(needs to be validated if bit 12 is 1, might need more operations to extract appropriate UTF8 sequence)

```
dep0_5[3] OR dep6_11[3]
OR dep12_15[3]          = 0000 0000 0001 1111       ..... (14)
dep0_5[5]               = 1000 0000 0100 0100       ..... (15)
(2) AND (15)            = 0000 0000 0000 0000       ..... (15)
~sur_sfx                = 1111 1111 1001 1111       ..... (16)
(15) OR (16)            = 1111 1111 1001 1111       ..... (17)
adv((17), 1)            = 1111 1111 1100 1111       ..... (18)
(18) AND (12)           = 1111 1111 1100 0111       ..... (19)
(14) AND (19)           = 0000 0000 0000 0111       ..... u8basis[3]
```

5. dep0_5[4], dep6_11[4], dep0_5[0] and dep6_11[5] are needed to calculate u8basis[4]. dep0_5[0] and dep6_11[0] are needed to extract bits 11 and 19 of 4-byte sequences.

```
dep0_5[0] OR dep6_11[0] = 1010 0010 0100 0110       ..... (20)
(2) AND (20)            = 0000 0000 0000 0000       ..... (21)
~(2) OR (21)            = 1111 1111 1100 1111       ..... (22)
dep0_5[4] OR dep6_11[4] = 0000 0100 0010 0110       ..... (23)
(22) AND (23)           = 0000 0100 0000 0110       ..... (24)
```
5.1 4th highest bit of 4-byte sequences shall be 1. 
Fetch the start byte positions of 3,4 byte sequences and use it to extract the start position of 4-byte sequences.
```
~dep6_11 AND dep12_15   = 0000 0001 0000 1001       ..... (25)
(10) AND (25)           = 0000 0000 0000 1000       ..... (26)
```

5.2 If first byte of 4-byte sequences is F0, its 5th highest byte must be 1.
```
u8basis[0] OR u8basis[1]
OR u8basis[2] OR
u8basis[3]              = 1111 1010 1100 0111       ..... (27)
~(27)                   = 0000 0101 0011 1000       ..... (28)
(28) AND (10)           = 0000 0000 0001 1000       ..... (29)
(24) OR (26) OR (29)    = 0000 0100 0001 1110       ..... u8basis[4]
```

6. dep0_5[5], dep6_11[5], dep0_5[1] and dep6_11[1] are needed to calculate u8basis[5]. dep0_5[1] and dep6_11[1] are needed to extract bits 10 and 18 of 4-byte sequences.

```
dep0_5[1] OR dep6_11[1]
AND (2)                 = 0000 0000 0000 0000       ..... (30)
~(2) OR (30)            = 1111 1111 1100 1111       ..... (31)
dep0_5[5] OR dep6-11[5]
AND (31)                = 1000 0010 0100 0110       ..... (32)
(25) OR (32)            = 1000 0011 0100 1111       ..... u8basis[5]
````

7. u8basis[6] indicates the first bytes of multibyte UTF-8 sequences or 6th bit of ASCII codepoints.
```
~dep0_5 AND dep12_15    = 0001 0001 0001 1001       ..... (33)
~(2) AND (33)           = 0001 0001 0000 1001       ..... (34)

ASCII = dep0_5 AND 
dep12_15                = 1100 1000 1000 0000       ..... (35)

ASCIIbit6 = dep6_11[0]
AND (35)                = 1000 0000 0000 0000       ..... (36)
(34) OR (36)            = 1001 0001 0000 1001       ..... u8basis[6]
```

8. u8basis[7] is essentially a stream of 1's for the codeunits which are non-ASCII.
```
nonASCII = ~ASCII       = 0011 0111 0111 1111       ..... u8basis[7]
```












