# Algorithm Description

The process of converting from CSV to JSON is broken down into steps:

1. Reading
2. Parsing
3. Compression
4. Template Creation
5. Expansion
6. Replacement
7. Writing

## Reading



## Parsing

**Input Streams:**

- **Byte Stream**- stream set containing a single 8-bit data streams- representing the characters of the CSV file.

**Output Streams:**

- **Basis Bits:** Stream set containing 8 1-bit data streams, representing the characters of the CSV file.
- **Double Quotes:** Stream set containing a single 1-bit data stream, indicating the positions in the CSV file that are non-escaped double quotes.
- **Delimiters:** Stream set containing a single 1-bit data stream, indicating the positions in the CSV file that are field delimiters.
- **Carriage Returns:** Stream set containing a single 1-bit data stream, indicating the positions in the CSV file that are syntactical carriage returns.
- **Line Feeds:** Stream set containing a single 1-bit data stream, indicating the positions in the CSV file that are syntactical line feeds.
- **Escaped Characters:** Stream set containing a 1-bit data stream, indicating the positions in the CSV file that are escaped double quotes, escaped carriage returns, or escaped line feeds.
- **Field Starts:** Stream set containing a single 1-bit data stream, indicating the first non-syntactical character in each field in Basis Bits.
- **Field Ends:** Stream set containing a single 1-bit data stream, indicating the last non-syntactical character in each field in Basis Bits.
- **Record Starts:** Stream Set containing a single 1-bit data stream, indicating the first non-syntactical character in each record in Basis Bits.
- **Record Ends:** Stream Set containing a single 1-bit data stream indicating the last non-syntactical character in each record in Basis Bits.

## Compression

**Input Streams:**

- **Basis Bits**
- **Outer Quotes**, **Delimiters**, **Carriage Returns**, **Line Feeds**
- **Escaped Characters**
- **Field Starts**, **Field Ends**
- **Record Starts**, **Record Ends**

**Internal Streams**:

- **Compression Mask**: Stream set containing a single 1-bit data stream, indicating the positions of all the syntactical characters in the CSV file, those which must be removed.

**Output Streams:**

- **Compressed Basis Bits:** Stream set containing 8 1-bit data streams, representing the data in the CSV file with the syntactical characters removed.
- **Insert Marks:**

## Template Creation

**Input Streams:**

**Output Streams:**

- **Template Vector:**

## Expansion

**Input Streams:**

- **Compressed Basis Bits**
- **Insert Marks**
- **Template Vector**

**Output Streams**:

- **Expanded Basis Bits:**
- **Expanded Insert Marks:**
- **Spread Mask:**

## Replacement

**Input Streams:**

- **Expanded Basis Bits**
- **Expanded Insert Marks**
- **Spread Mask**

**Internal Streams:**

- **Character Indices:**

**Output Streams:**

- **Translated Basis Bits**

## Writing

