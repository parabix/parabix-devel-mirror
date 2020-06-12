# TO DO:

1. Implement kernel to remove CRs from file BEFORE doing any other processing. This will do a lot to prevent errors.
2. Update requirements to state: a. no CR's in output, b. no escaped characters in header
3. Implement escaping for backslashes.
4. Move header and bixnum_size parameters to fn(), rather than generatepipelinefunction.
5. Implement multi-file input.
6. What are algorithmic notes?
7. Update test script using xml. Return to code?
8. Throw error message when input file is incorrectly formatted.
9. Support for tab delimited values.
10. Support for user set fieldnames.
11. Testcase_escapes
12. Debug testcase_doublequote