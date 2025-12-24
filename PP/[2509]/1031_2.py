def reverse_sublist(lst, l, r, inplace):
    if l>=r or l<0 or r>len(lst):
        raise AssertionError('Invalid sublist')
    lst_1=lst[0:l]
    lst_2=lst[l:r]
    lst_3=lst[r:len(lst)]
    lst_2=lst_2[::-1]
    if inplace==False:
        lst__=lst_1+lst_2+lst_3
        return lst__
    elif inplace==True:
        lst[:]=lst_1+lst_2+lst_3
        return lst
def rotate_sublist(lst, l, r, k, inplace):
    if l>=r or l<0 or r>len(lst):
        raise AssertionError('Invalid sublist')
    n=r-l
    k=k%n
    lst_1=lst[0:l]
    lst_2=lst[l:l+k]
    lst_3=lst[l+k:r]
    lst_4=lst[r:len(lst)]
    if inplace==False:
        lst__=lst_1+lst_3+lst_2+lst_4
        return lst__
    elif inplace==True:
        lst[:]=lst_1+lst_3+lst_2+lst_4
        return lst
def swap_sublist(lst, l1, r1, l2, r2, inplace):
    if not (0 <= l1 < r1 <= len(lst) and 0 <= l2 < r2 <= len(lst)):
        raise AssertionError('Invalid sublist')
    if l1>l2:
        l1, r1, l2, r2 = l2, r2, l1, r1
    if r1 > l2:
        raise AssertionError('Invalid sublist')
    lst_0=lst[0:l1]
    lst_1=lst[l1:r1]
    lst_2=lst[r1:l2]
    lst_3=lst[l2:r2]
    lst_4=lst[r2:len(lst)]
    if len(lst_1)!=len(lst_3):
        raise AssertionError('Invalid sublist')
    if inplace==False:
        lst__=lst_0+lst_3+lst_2+lst_1+lst_4
        return lst__
    elif inplace==True:
        lst[:]=lst_0+lst_3+lst_2+lst_1+lst_4
        return lst