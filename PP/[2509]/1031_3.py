import math
lines=int(input())
LST=[]
for i in range(lines):
    lst=input().split()
    LST.append(lst)
#print(LST)
def count(t,lst):
    num=0
    for i in lst:
        if i==t:
            num+=1
    return num
#print(count('oo',lst))
def Term_Frequency(t,lst):
    TF=math.log10(count(t,lst)+1)
    return TF
#print(Term_Frequency('oo',lst))
def Document_Frequency(t,LST):
    DF=0
    for i in LST:
        if t in i:
            DF+=1
    return DF
#print(Document_Frequency('miku',LST))
def Inverse_Document_Frequency(t):
    IDF=math.log10(lines/Document_Frequency(t,LST))
    return IDF
#print(Inverse_Document_Frequency('miku'))
def TF_IDF(t,i):
    TF_IDF=0
    if t in LST[i]:
        TF_IDF=Term_Frequency(t,LST[i])*Inverse_Document_Frequency(t)
    return TF_IDF
#print(TF_IDF('miku',0))
#用这个我能算出任意一个t在i+1行的TF_IDF
for i in range(lines):
    k1=LST[i][0]
    k2=0
    for t in LST[i]:
        if TF_IDF(t,i)>k2:
            k1=t
            k2=TF_IDF(t,i)
    f2=f"{k2:.4f}"
    print(k1,f2)