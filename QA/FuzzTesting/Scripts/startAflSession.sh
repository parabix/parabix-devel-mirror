#!/bin/sh
AFL_SKIP_CPUREQ=1 ../QA/FuzzTesting/AFL-Files/afl-fuzz -m 100 -i ../QA/FuzzTesting/afl-in/ -o out/ -- ./icgrep -f @@ ../QA/FuzzTesting/FuzzIcgrepInput 

