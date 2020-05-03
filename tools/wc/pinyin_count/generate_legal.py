#generate all possible syllables
initial=["","b","p","m","f","d","t","n","l","g","k","h","j","q","x","zh","ch","sh","r","z","c","s","y","w"]
final=["","i","a","o","e","e_hat","ai","ei","ao","ou","an","en","ang","eng","ong","er","i","ia","ie","iao","iu","ian","in","ing","iang","iong","u","ua","uo","uai","ui","uan","un","uang","ueng","v","ve","van","vn",]
output=open("legal_syllables.py","w+")
possible=[]
for i in initial:
    for j in final:
        possible.append(i+j)

#iterate over possible syllables and record legal syllables
legal=[]
lines=[]
khanyu_path='/home/wendy/parabix-devel/include/unicode/data/KHanyuPinyin.h'
f=open(khanyu_path,"r")
for line in f:
    lines.append(line)

for pos in possible:
    test="    const static std::array<UnicodeSet, 5> "+pos+"_Set = {\n"
    if test in lines:
        legal.append(pos)

output.writelines("legal=")

temp=str(legal)
temp=temp.replace("'",'"')
table=list(temp.split(','))
count=0
for entry in table:
    if entry==table[-1]:
        output.write(entry)
        continue
    output.write(entry+',')
    count+=1
    if (count>=24):
        output.write('\n')
        count=0