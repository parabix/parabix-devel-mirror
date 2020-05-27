# module: beta-csv

parabix_add_executable(
NAME
	csv2json
SRC
    	csv2json_2.0.cpp
	#csv2json.cpp
	
DEPS
    	pablo
    	kernel.basis
    	kernel.io
    	kernel.pipeline
    	kernel.streamutils
    	kernel.util
    	toolchain
    	fileselect
    	re.cc
    	re.parse
)



parabix_add_executable(
NAME
    SimpleCheck
SRC
    SimpleCheck.cpp
DEPS
    
)

