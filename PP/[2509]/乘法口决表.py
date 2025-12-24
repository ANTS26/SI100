num=float(input())
table=('')

if int(num) != num or num<1 or num>9:
    print("Invalid")

else:
    for i in range (1,int(num)+1):
        for j in range (1,i+1):
         if j == 1:
            table+=f"{j}x{i}={j*i:2}"
         else:
            table+=f"\t{j}x{i}={j*i:2}"
        if j == i:
            table+='\n'
print(table)