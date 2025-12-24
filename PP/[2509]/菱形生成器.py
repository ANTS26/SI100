a=int(input())
b=int(input())
c=input()
table=''
num_max=b*(a//2)+1
for i in range (a):
    row_max=a//2
    dis=abs(i-row_max)
    num_now=num_max-dis*b
    num_space=(num_max-num_now)//2
    table+=' '*num_space+c*num_now+' '*num_space+'\n'
print(table)