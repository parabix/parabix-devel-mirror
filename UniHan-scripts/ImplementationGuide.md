## Implementation Guide
THIS is the implementation guide for `Unihan_scripts`.
`Unihan_scripts` is modified from `UCD-scripts`. We inherited the use of `unicode_set` data structure, which is aligning the `UCD::UnicodeSet` definition in Parabix. Also, considering that we only need limited number of fields in Unihan datafiles, we simplified the scripts by removing the use of `property_object`.

### The procedure of KPY or KXHC generation
1. Parse each line in `Unihan_Reading.txt`. Deal with a line if and only if it contains the fields(`kHanyuPinyin`, `kXHC1983`) we want to process.
2. Get the codepoint as well as the corresponding pinyin readings. If any pinyin reading has not been encountered, add a new property value(e.g. `zhong`) to the `value_map`. Merge this codepoint with the `unicode_set` in the map entries corresponding to the syllables. Array is used to make the tone of syllables more explicit. 
3. Output the result to a c/cpp header file.

### The procedure of KTraditional generation
1. Parse each line in `Unihan_Variant.txt`. Deal with a line if and only if it contains the fields(`kTraditionalVariant`, `kSimplifiedVariant`) we want to process.
2. Two property value `sim_only` for simplified only and `trd_only` for traditional only are predefined, while two `unicode_set` containing the codepoint having `kTraditionalVariant` or `kSimplifiedVariant` fields. Codepoints are gathered for these two `unicode_set` line by line.
3.  ```python
    value_map['sim_only'] = uset_intersection(uset_complement(have_simplified), have_traditional)
    value_map['trd_only'] = uset_intersection(uset_complement(have_traditional), have_simplified)
    ```
    Corresponding map entries are created and output.
