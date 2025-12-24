table=[]
for i in range(5):
    line=input()
    table.append(line)
x1=float(table[0])
y1=float(table[1])
x2=float(table[2])
y2=float(table[3])
x=float(table[4])
y=y1 + (y2-y1)/(x2-x1)*(x-x1)
print(f'{y:.2f}')