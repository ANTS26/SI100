def get_op(op):
    if op =="add":
        return lambda x,y: x + y
    elif op == "sub":
        return lambda x,y: x - y
    elif op == "mul":
        return lambda x,y:x*y
    elif op == "div":
        return lambda x,y:x/y
    elif op == "max":
        return lambda x,y: max(x, y)
    elif op == "pow":
        return lambda x,y: x ** y
    elif op =="min":
        return lambda x,y:min(x,y)
def calc(op,a,b):
    return op(a,b)