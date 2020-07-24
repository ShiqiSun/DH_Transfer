import DH_tranfer as dh
import jacobi as jb
import numpy as np
import math as math

thetas = [0, 0, math.radians(90), 0, 0, 0]
base = [0, 0, 0]
target = [3, 1, 1]
basep = np.array([[0], [0], [0], [1]])


dht = dh.RobotArmModel_Feedforward(thetas)
print(dht)
dht_0 = np.zeros((4, 4))
dht_0[:, 3] = [0, 1, -2, 1]

# x, y = jb.ik(dht_0)
z = dht_0.dot(basep)
# print(y)
print(z)













