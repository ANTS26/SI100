#一，简易运算
i,j=2,3
a=i//j #取整除
a=i%j #取余数
a=i**j #幂运算,开方
a-=i #a=a-i
i='abcdefgh',j='ijk',i+' '+j='abcdefgh ijk' #字符串连接
i[0]='a',i[-1]='h'#字符串取值
i[3:6]='def',i[3:6:2]='df',i[:]='abcdefgh',i[::-1]='hgfedcba',i[4:1:-2]='ec'#字符串切片

#二，输入输出
a=input().split()#接收多个输入，默认以空格分隔，返回list。如果是，split(',')，则以逗号分隔
print(f'{a*i} is {i*j} % of {j}') #格式化输出
print(f'{a:.2f}') #保留两位小数,注意：type(f'{a:.2f}')=str，所以不能进行数学运算
print(''.join([lst1]))#将列表lst1元素合并输出，无分割。返回str。''.join(['1','2','3’]。如果是'k'.join([lst1])，则以k分隔
from pprint import pprint
pprint(lst)#多层表格，自动对齐

#三，if while for try
#1，if
if a>i and a!=0:
#a=True,b=False,a and b=False,a or b=True,not a=False #布尔运算
    print('a>i')
elif a==i:
    raise ValueError('不对')#抛出异常，知道是哪种报错.
#ValueError值错误,TypeError类型错误,IndexError索引错误,KeyError键错误,AttributeError属性错误,ZeroDivisionError除零错误,ImportError导入错误,等等
else:
    raise AssertionError('a<i')#抛出异常，断定错了，但是不知道是哪种错
#2，while
while True:
    if type(input())!=str:
        a+=int(input())
    else:
        break#多用于持续接收输入
#3，for
l=['a','b','c','d']
for i in range(len(l)):#从0到len(l)-1
    print(i, l[i])
for i in l:
    print(a,i)
    a+=1#两者等价，多用于遍历列表
for i in range(2,10,2):#从2到9，步长为2,左闭右开
    print(i)#输出2,4,6,8 代表内容而不是序数
    continue#跳过本次循环，进行下一次循环
    break#结束循环

#try except
try:
    a=int(input())
    b=int(input())
    print(a/b)
except ZeroDivisionError:
    print("除零错误")
except Exception as e:
    print(f"其他错误: {e}")
except :
    print("未知错误")

#四，函数
def func_name(a,b):
    sum=0
    for i in range(a,b+1):#因为左闭右开，所以b+1，才能遍历完全
        sum+=i
    return sum
def do_twice(n, fn):#参数可以为函数
    return fn(fn(n))
print(do_twice(3, lambda x: x**2))#lambda匿名函数,输出81

#五，tuple,list&dict
#1，tuple不可变
tuple_short = (2,) #单元素tuple需要逗号
tuple1 = (1, 2, 3, 4, 5, 6)
print(tuple1[4:1:-1]) #(5, 4, 3)倒序切片
#2，list，可变
lst1=[1,2,3,4,5],lst2=['a','b','c']
lst1[1:4] = [6,7,8] #切片修改
lst3=lst1 + lst2 #列表连接[1,6,7,8,5,'a','b','c']
Lnew =[e**2 for e in lst1 if e%2==0] #语法糖，生成新列表，Lnew为L中所有偶数的平方

lst4.append(lst1,lst2) #元素添加lst4=[[1,6,7,8,5],['a','b','c']]
lst2.remove('b') #按值删除
lst2.pop(0) #按索引删除,第一个。del lst1[2] #按索引删除，第三个
lst2.sort() #排序，默认升序,lst2.sort(reverse=True) #降序
lst2.reverse() #列表反转
lst2.clear() #清空列表，变成[]，但是Id不变，和lst2=[]不同

#浅拷贝（无论是哪个拷贝建议都对lst_copy进行操作，而不是原lst，这样更安全）
lst=[[1,2],3,4]
lst_copy=lst[:]
lst_copy[0].reverse()#对深层操作会变
print(lst,lst_copy)#[[2,1],3,4] [[2,1],4,3]，lst也变。注意，这边外层之所以会变
lst_copy.reverse()#只对最外层操作不会变
print(lst,lst_copy)#[[1,2],3,4] [4,3,[1,2]]，lst未变
#深拷贝
import copy
lst_deepcopy=copy.deepcopy(lst)
lst_deepcopy[0].reverse()#对深层操作也不会变
print(lst,lst_deepcopy)#[[1,2],3,4] [[2,1],3,4]，lst未变

#3，dict字典
a=['Matt','John','Katy','Grace']
b=[100,95,87,88]
grades=dict(zip(a,b))#创建字典,直接grades={}也可以
grades['KK']=101#增加元素
grades['Grace']=99#修改元素
del(grades['John'])#删除元素
print(list(grades.items()))#[('Matt', 100), ('Katy', 87), ('Grace', 99), ('KK', 101)]
print(list(grades.keys()))#['Matt', 'Katy', 'Grace', 'KK']
print(list(grades.values()))#[100, 87, 99, 101]
def count_matches(d):#字典用法
    count=0
    for key in d :
        if key==d[key]:
            count+=1
    return count
#六，递归法
def count_elem(L):
    if not isinstance(L, list):
        return 1
    total = 0
    for item in L:
        total += count_elem(item)
    return total
print(count_elem([1, 2, [[3, 4], 5]]))
#简单来说就是，else部分放要递归的操作，if部分放最开始的部分

#N+1，其他
#1，浮点精度问题
for i in range(10): 
    x += 0.125
    y += 0.1
print(x == 1.25)#True，float到float
print(y == 1.0)#False，float到int

#N,基础算法
#1，穷举法计算平方根 Guess and Check
while abs(guess**2- x) >= epsilon and guess**2 <= x:
        guess += increment
        numGuesses += 1
print('numGuesses =', numGuesses)
if abs(guess**2- x) >= epsilon:
    print('Failed on square root of', x)
else:
    print(guess, 'is close to square root of', x)
#2，二分查找法计算平方根 Bisection Search
while abs(guess**2- x) >= epsilon:
    if guess**2 < x :
        low = guess #把low抬高
    else:
        high = guess#把high降低
    guess = (high + low)/2.0
    num_guesses += 1 
print('num_guesses =', num_guesses)
#3，牛顿法计算平方根 Newton-Raphson Method
while abs(guess**2 - x) >= epsilon:
    guess = guess - (((guess**2) - x)/(2*guess))
    #x2=x1-f(x1)/f'(x1)，希望找到f（xn）=0。故此处f（g）=g^2 - x ，f'(g)=2g
    num_guesses += 1
print('num_guesses =', num_guesses)