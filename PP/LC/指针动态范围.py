class Solution:
    def lengthOfLongestSubstring(s: str) -> int:
        if not s:return 0
        left = 0
        lookup = []
        n = len(s)
        max_len = 0
        cur_len = 0
        for i in range(n):
            cur_len += 1
            while s[i] in lookup:
                lookup.remove(s[left])
                left += 1
                cur_len -= 1
            if cur_len > max_len:max_len = cur_len
            lookup.append(s[i])
        return max_len
    #动态范围：pwwkew,一点一点左移扩大lookup，如果碰到s[i] in lookup就从左边减掉lookup，然后指针右移一位。一直减到没有为止，这时候右边接着加
    print(lengthOfLongestSubstring("pwwkew"))