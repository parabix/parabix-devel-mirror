#generate all possible syllables+tone
from legal_syllables import legal

output=open("unicodeset_table.txt","w+")
lines=[]

khanyu_path='/home/wendy/parabix-devel/include/unicode/data/KHanyuPinyin.h'
f=open(khanyu_path,"r")
for line in f:
    lines.append(line)

table=[]

for syllable in legal:
    for i in range(6):
        x=str(i)
        test="    const static UnicodeSet::run_t __"+syllable+x+"_Set_runs[] = {\n"
        if test in lines:
            #< <jing,1>, &jing_Set[1] >
            temp=('{make_pair("'+syllable+'",'+x+"),&"+syllable+"_Set["+x+"]}")
            temp+=","
            table.append(temp.ljust(40))

#get rid of last comma
table[-1]=table[-1].strip(' ')
table[-1]=table[-1][:-1]


count=0
for entry in table:
    if entry==table[-1]:
        output.write(entry)
        continue
    output.write(entry)
    count+=1
    if (count>=6):
        output.write('\n')
        count=0

output.close()

