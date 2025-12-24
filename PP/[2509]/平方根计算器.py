a=float(input())
guess=a/2
for i in range(200):
    guess = (guess + a / guess) / 2
print(f"{guess:.20f}")