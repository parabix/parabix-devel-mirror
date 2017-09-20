Generating UCD data files.

1. Edit UCD_config.py - set UCD_src_dir, UCD_output_dir

2. UnicodeNameData
   python UnicodeNameData.py
   
3. python UCD_properties.py

4. python casefold.py

5. python generate_UCD_tests.py
    copy output to icgrep-devel/QA/proptest.xml

