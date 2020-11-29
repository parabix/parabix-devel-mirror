Generating UCD data files.

1. Edit UCD_config.py - set UCD_src_dir, UCD_output_dir
   Make sure that emoji files have been placed in the emoji subdirectory of the UCD_src_dir
   Create the UCD_output_dir

2. python3 UCD_properties.py
   copy generated UCD_Config.h to include/unicode/core
   copy other generated files to include/unicode/data
3. python3 UCD_equivalence.py
   copy generated Equivalence.cpp to lib/unicode/data
   copy generated PrecomposedMappings.h  to include/unicode/data
4. python3 generate_UCD_tests.py > proptest.xml
   copy proptest.xml to the QA directory
