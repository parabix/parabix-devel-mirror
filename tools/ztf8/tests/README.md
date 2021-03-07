# Phrase based ztf-hash 

### Step 1: Unicode word segmentation of the input data
* Given input is segmented into a list of individual words/symbols in accordance with the Unicode word segmentation algorithm.

  * **Example:** 
  Input text - The Project Gutenberg Etext of The 1913 Webster Unabridged Dictionary
  * Unicode words - ['\n', 'The', ' ', 'Project', ' ', 'Gutenberg', ' ', 'Etext', ' ', 'of', ' ', 'The', ' ', '1913', ' ', 'Webster', ' ', 'Unabridged', ' ', 'Dictionary'] 
* Based on the maximum number of words to be considered in a phrase while compression, the input text is split into appropriate phrases. We are considering maximum 4 words in a phrase now. To start with, the phrases look like this.

`('\n', 'The', ' ', 'Project'),`
`('The', ' ', 'Project', ' '),`
`(' ', 'Project', ' ', 'Gutenberg'),`
`('Project', ' ', 'Gutenberg', ' '),`
`(' ', 'Gutenberg', ' ', 'Etext'),`
`('Gutenberg', ' ', 'Etext', ' '),`
`(' ', 'Etext', ' ', 'of').`
`('Etext', ' ', 'of', ' '),`
`(' ', 'of', ' ', 'The'),`
`('of', ' ', 'The', ' '),`
`(' ', 'The', ' ', '1913'),`
`('The', ' ', '1913', ' '),`
`(' ', '1913', ' ', 'Webster'),`
`('1913', ' ', 'Webster', ' '),`
`(' ', 'Webster', ' ', 'Unabridged'),`
`('Webster', ' ', 'Unabridged', ' '),`
`(' ', 'Unabridged', ' ', 'Dictionary')`

### Step 2: Check for longest phrase to be encoded in a hierarchical manner of number of words in a phrase

* Once all the word phrases of 4 bytes are identified, start checking each phrase to have already occurred in the text parsed and encoded so far. We'll have 2 cases to handle:
    1. If the phrase was already seen and has an entry for corresponding codeword in the hash table: if this is the case, we would directly replace the phrase with a codeword of two bytes. The steps of calculating codeword of a word is inspired by BixHash kernel of ztf-hash application and will be explained in a separate section.
    2. If the phrase is seen for the first time in the text, we would calculate the corresponding codeword value for that phrase and save it in one of the hash tables based on the length of the phrase.
       * Once the codeword for the phrase is calculated, we add the first word of the phrase to a fallBack array to check if it can be encoded together in the final compressed text.
       * The fallback mechanism continues until all the words of the longer phrase are compressed either in smaller group of words or individual words. Only the phrases/ words of length > 2 are compressed as the maximum codeword length currently being handled is 2 bytes.
    > The hash tables are separate for phrases of different length. If the codeword for two different phrases of different length is same, in order to utilize the opportunity of compressing as many as possible phrases, we keep the phrases (key) and their codewords (values) in different hash tables.

TODO: Evaluation of codeword space expansion
### Hash calculation:





