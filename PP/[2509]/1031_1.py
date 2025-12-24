l=[]
def flatten_nested_list(nested):
    for i in nested:
        if type(i)==list:
            i=flatten_nested_list(i)
        if type(i)!=list:
            l.append(i)
    return l