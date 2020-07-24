import numpy as np
import math
from Constant import *


def DH_Matrix(alpha, a, theta, d):
    theta_r = math.radians(theta)
    alpha_r = math.radians(alpha)
    dh_matrix = np.zeros((4, 4))

    dh_matrix[0, 0] = math.cos(theta_r)
    dh_matrix[0, 1] = -1*math.sin(theta_r)*math.cos(alpha_r)
    dh_matrix[0, 2] = math.sin(theta_r)*math.sin(alpha_r)
    dh_matrix[0, 3] = a*math.cos(theta_r)

    dh_matrix[1, 0] = math.sin(theta_r)
    dh_matrix[1, 1] = math.cos(theta_r)*math.cos(alpha_r)
    dh_matrix[1, 2] = -1*math.cos(theta_r)*math.sin(alpha_r)
    dh_matrix[1, 3] = a*math.sin(theta_r)

    dh_matrix[2, 0] = 0
    dh_matrix[2, 1] = math.sin(alpha_r)
    dh_matrix[2, 2] = math.cos(alpha_r)
    dh_matrix[2, 3] = d

    dh_matrix[3, 0] = 0
    dh_matrix[3, 1] = 0
    dh_matrix[3, 2] = 0
    dh_matrix[3, 3] = 1

    return dh_matrix


def RobotArmModel_Feedforward(thetas):
    if len(thetas) != 6:
        print("The number of angles is wrong!")
        return np.zeros((4, 4))

    dh_1 = DH_Matrix(0, 0, thetas[0], l1)
    dh_2 = DH_Matrix(-90, 0, thetas[1], 0)
    dh_3 = DH_Matrix(0, l2, thetas[2], 0)
    dh_4 = DH_Matrix(0, l3, thetas[3], 0)
    dh_5 = DH_Matrix(90, 0, thetas[4], l4)
    dh_6 = DH_Matrix(-90, l5, thetas[5], 0)

    dh_list = [dh_1, dh_2, dh_3, dh_4, dh_5, dh_6]

    dh_transfer = dh_1
    for i in range(len(dh_list)-1):
        dh_transfer = dh_transfer.dot(dh_list[i+1])

    return dh_transfer


def FeedForwardPositionTransfer(base_coord, thetas):
    origin_position = np.zeros((4, 1))
    origin_position[3, 0] = 1

    for i in range(len(base_coord)):
        origin_position[i, 0] = base_coord[i]

    dh_table = RobotArmModel_Feedforward(thetas)
    End_position = dh_table.dot(origin_position)

    End_coord = []
    for i in range(3):
        End_coord.append(End_position[i, 0])

    return End_coord