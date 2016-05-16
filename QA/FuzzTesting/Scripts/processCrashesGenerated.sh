#!/bin/sh
# This scripts archives the previous AFL sessions output files as well as extracts the crashes for Jenkins Artifact archiving.

#move all the files previous session generated.
newName=OldResults/out.$(date "+%F-%T")
mv out ${newName}

#Archive the crashes files we want.
rm -r ../../../IcGrep-FuzzyTesting/workspace/Artifacts/*
mkdir ../../../IcGrep-FuzzyTesting/workspace/Artifacts/*
mv  ${newName}/crashes/* ../../../IcGrep-FuzzyTesting/workspace/Artifacts/

return 0
