from subprocess import *
from string import *

if __name__ == "__main__":
	out_f = open('OutputFiles/reads2', 'w')
	expected_f = open('ExpectedOutput/reads2', 'r')
	call(['../../editd-build/editd', '-f', 'TestFiles/reads2', 'chr.fa', '-display'], stdout=out_f)
	if open('OutputFiles/reads2', 'r').read()==expected_f.read():
		print 'Succeeded'
	else:
		print 'Failed'	
