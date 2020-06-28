# TO DO:

**Documentation To Do List**

1. Update requirements to state: a. no CR's in output, b. no escaped characters in header.
2. Write project report pdf.
3. Write documentation for the new implementation features mentioned in the Implementation to do List below. 
4. Make project report .ppt for out 15-20 min presentation.
5. Make QA/csv2json/outs not tracked by git and delete its contents from the repo.



**Project Report Requirements**

I have had questions about the structure of the final project report.

This report should be a structured report of 600-1000 and should have 4 sections.
    Introduction.
    Project description and operation.
    Software engineering issues discussion.
    Concluding remarks and future work.

The project description and operation should illustrate the basic
concept of the application and how it may be used.  It should
include illustrative examples.  However, it is not a user manual
or programmer documentation, which should continue to be included
in the source code repository.

The section on software engineering issues should address the challenges
that you encountered in building your application and how you met those
challenges.  This includes the complexity of dealing with a somewhat
large and unknown code base, working with GitLab and cmake, as well
as issues in teamwork and coordiation.

The concluding section of your report should highlight the most
important things that you have learned as well as outlining possilbe future
directions for work to enhance your applications.

**Implementation To Do List**

1. ~~Implement kernel to remove CRs from file BEFORE doing any other processing. This will do a lot to prevent errors.~~
2. ~~Command line option for processing all .csv files in a directory.~~
3. ~~Support for user set fieldnames.~~
4. ~~Support for tab delimited values.~~
5. ~~Implement escaping for backslashes.~~
6. ~~Testcase_escapes~~
7. ~~Debug testcase_doublequote~~
8. Support for user specification of field type and NULL values for empty fields.

