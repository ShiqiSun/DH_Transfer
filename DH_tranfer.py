import numpy as np
import math

def DH_Matrix(alpha, a, theta, d):
    theta_r = math.radians(theta)
    alpha_r = math.radians(alpha)
    DH_matrix = np.zeros((4, 4))

    DH_matrix[0, 0] = math.cos(theta_r)
    DH_matrix[0, 1] = -1*math.sin(theta_r)*math.cos(alpha_r)
    DH_matrix[0, 2] = math.sin(theta_r)*math.sin(alpha_r)
    DH_matrix[0, 3] = a*math.cos(theta_r)

    DH_matrix[1, 0] = math.sin(theta_r)
    DH_matrix[1, 1] = math.cos(theta_r)*math.cos(alpha_r)
    DH_matrix[1, 2] = -1*math.cos(theta_r)*math.sin(alpha_r)
    DH_matrix[1, 3] = a*math.sin(theta_r)

    DH_matrix[2, 0] = 0
    DH_matrix[2, 1] = math.sin(alpha_r)
    DH_matrix[2, 2] = math.cos(alpha_r)
    DH_matrix[2, 3] = d

    DH_matrix[3, 0] = 0
    DH_matrix[3, 1] = 0
    DH_matrix[3, 2] = 0
    DH_matrix[3, 3] = 1