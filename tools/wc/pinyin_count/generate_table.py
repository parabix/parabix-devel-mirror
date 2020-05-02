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
            table.append("<<"+syllable+","+x+">,&"+syllable+"_Set["+x+"]>")

output.writelines(str(table)[1:-1])
output.close()

