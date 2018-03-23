import numpy as np
import math as math

a = np.array([[207025,455], [164025,405]])
b = np.array([0, 235])
x = np.linalg.solve(a, b)
 
print (-x[0]*1000)
print (math.degrees(math.atan(x[1])))
