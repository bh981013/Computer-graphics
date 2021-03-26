import numpy as np

#A
M = np.arange(2, 27)
print(M)
print()

#B
M = M.reshape(5,5)
print(M)
print()

#C
M[:,0] = 0
print(M)
print()

#D
M = M@M
print(M)
print()

#E
v = M[0]
result = np.sqrt(sum(v*v))
print(result)
