fr = open("unicodeset_radical_table.txt", "w")

lines = []
krskangxi_path = '../../../include/unicode/data/kRSKangXi.h'
f = open(krskangxi_path, "r")

for l in f:
    lines.append(l)
    
table = []

for i in range(1, 215):
    code = str(i)
    search = "    const static UnicodeSet::run_t __" + "_" + code + "_Set_runs[] = {\n"
    if search in lines:
        temp = ("{" + "\"" + code +  "\"" + ",&_" + code + "_Set}")  #{"74",&_74_Set}
        temp = temp + ","
        table.append(temp.ljust(35))
        
table[-1] = table[-1].strip(' ')
table[-1] = table[-1][:-1]

c=0
for i in table:
    if i == table[-1]:
        fr.write(i)
        continue
    fr.write(i)
    c = c + 1
    if (c >= 6):
        fr.write('\n')
        c = 0

fr.close()
