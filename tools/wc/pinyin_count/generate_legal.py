#generate all possible syllables
initial=["","b","p","m","f","d","t","n","l","g","k","h","j","q","x","zh","ch","sh","r","z","c","s","y","w"]
final=["i","a","o","e","ai","ei","ao","ou","an","en","ang","eng","ong","er","i","ia","ie","iao","iu","ian","in","ing","iang","iong","u","ua","uo","uai","ui","uan","un","uang","ueng","v","ve","van","vn","e_hat"]
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
output.writelines(str(legal))
output.close()
