Generating UCD data files.

1. Edit UCD_config.py - set UCD_src_dir, UCD_output_dir
   Make sure that emoji files have been placed in the emoji subdirectory of the UCD_src_dir

2. python3 UCD_properties.py
3. python3 UCD_equivalence.py

#  Needs to be updated, 
4. python3 generate_UCD_tests.py
    copy output to icgrep-devel/QA/proptest.xml

