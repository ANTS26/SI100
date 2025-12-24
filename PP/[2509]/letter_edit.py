s=''
s=s.strip()#去除首尾空格
s=s.lower()#全部转小写
#s=s.upper()#全部转大写

def despace(s):
    for i in s:
        if i==' ':
            s=s.replace(i,'')
    return s
s=despace(s)#去除空格

def depoint(s):
    for i in s:
        if i.isalpha()==True or i.isdigit()==True:
            continue
        else:
            s=s.replace(i,'')
    return s
s=depoint(s)#去除句点

def denum(s):
    for i in s:
        if i.isdigit()==True:
            s=s.replace(i,'')
    return s
s=denum(s)#去除数字

#使用范例
#a=input()
#a=letter_edit.despace(a)
#a=letter_edit.depoint(a)
#a=letter_edit.denum(a)
#a=a.lower()