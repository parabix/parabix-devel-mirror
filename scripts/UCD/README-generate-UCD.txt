Generating UCD data files.

1. Edit UCD_config.py - set the Unicode version
   Ensure that the Unicode database is placed in the UCD_src_dir
   Make sure that emoji files have been placed in the emoji subdirectory of the UCD_src_dir

2. python3 UCD_properties.py
3. python3 UCD_equivalence.py
4. copy generated include/UCD_Config.h to include/unicode/core
   copy other generated include files to include/unicode/data
   copy generated src files to lib/unicode/data

5. python3 generate_UCD_tests.py > proptest.xml
   copy proptest.xml to the QA directory
