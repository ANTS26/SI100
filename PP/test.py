class Solution(object):
    def isValid(s):
        """
        :type s: str
        :rtype: bool
        """
        l=[]
        for i in s:
            if i=='('or i=='['or i=='{':
                l.append(i)
            if i==')' and l!=[] and l[-1]=='(':
                l.pop(-1)
            if i==']' and l!=[] and l[-1]=='[':
                l.pop(-1)
            if i=='}' and l!=[] and l[-1]=='{':
                l.pop(-1)          
        if l!=[]:return 'flase'
        else: return 'ture'
    print(isValid('()'))