from math import *
from Constant import *
import numpy as np
import random


def ArmCheck(theta1, theta2, theta3):
    theta1r = radians(theta1)
    theta2r = radians(theta2)
    theta3r = radians(theta3)
    y1 = l1 + l2*sin(theta1r)
    y2 = l1 + l2*sin(theta1r) + l3*sin(theta1r+theta2r)
    y3 = l1 + l2*sin(theta1r) + l3*sin(theta1r+theta2r) + l4*sin(theta1r+theta2r+theta3r)
    y4 = l1 + l2*sin(theta1r) + l3*sin(theta1r+theta2r) + l4*sin(theta1r+theta2r+theta3r) - l5*cos(theta1r+theta2r+theta3r)
    if y1 < 0 or y2 < 0 or y3 < 0 or y4 < 0:
        return False
    else:
        return True


def getxy(theta1, theta2, theta3):
    theta1r = radians(theta1)
    theta2r = radians(theta2)
    theta3r = radians(theta3)
    # x = l2*cos(theta1r) + l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r)
    # y = l2*sin(theta1r) + l3*sin(theta1r+theta2r) + l4*sin(theta1r+theta2r+theta3r)
    x = l2*cos(theta1r) + l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r) + l5*sin(theta1r+theta2r+theta3r)
    y = l1 + l2*sin(theta1r) + l3*sin(theta1r+theta2r) + l4*sin(theta1r+theta2r+theta3r) - l5*cos(theta1r+theta2r+theta3r)
    return x, y


def jacbixy(theta1, theta2, theta3, deltax, deltay):
    theta1r = radians(theta1)
    theta2r = radians(theta2)
    theta3r = radians(theta3)
    ja = np.zeros((2, 3))
    # print(theta1r, theta2r, theta3r)
    # ja[0, 0] = - l2*sin(theta1r) - l3*sin(theta1r+theta2r) - l4*sin(theta1r+theta2r+theta3r)
    # ja[0, 1] = -  l3*sin(theta1r+theta2r) - l4*sin(theta1r+theta2r+theta3r)
    # ja[0, 2] = - l4*sin(theta1r+theta2r+theta3r)
    # ja[1, 0] = l2*cos(theta1r) + l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r)
    # ja[1, 1] = l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r)
    # ja[1, 2] = l4*cos(theta1r+theta2r+theta3r)
    ja[0, 0] = - l2*sin(theta1r) - l3*sin(theta1r+theta2r) - l4*sin(theta1r+theta2r+theta3r) + l5*cos(theta1r+theta2r+theta3r)
    ja[0, 1] = -  l3*sin(theta1r+theta2r) - l4*sin(theta1r+theta2r+theta3r) + l5*cos(theta1r+theta2r+theta3r)
    ja[0, 2] = - l4*sin(theta1r+theta2r+theta3r) + l5*cos(theta1r+theta2r+theta3r)
    ja[1, 0] = l2*cos(theta1r) + l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r) + l5*sin(theta1r+theta2r+theta3r)
    ja[1, 1] = l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r) + l5*sin(theta1r+theta2r+theta3r)
    ja[1, 2] = l4*cos(theta1r+theta2r+theta3r) + l5*sin(theta1r+theta2r+theta3r)

    # x = deltax / (abs(deltax) + abs(deltay))
    # y = deltay / (abs(deltax) + abs(deltay))
    # x = deltax/abs(deltax) * np.square(deltax) / np.square(deltax) + np.square(deltay)
    # y = deltay/abs(deltay) * np.square(deltay) / np.square(deltax) + np.square(deltay)
    # ja[0, 0] = j00*x + j10*y
    # ja[1, 0] = j01*x + j11*y
    # ja[2, 0] = j02*x + j12*y
    # ja[0, 0] = j00 + j10
    # ja[1, 0] = j01 + j11
    # ja[2, 0] = j02 + j12
    # print(ja)
    return ja


def iter(theta1, theta2, theta3, goalx, goaly):
    error = 1e-8
    iter = 0
    step = 0.5
    # print(goalx, goaly)
    while iter < 10000:
        x, y = getxy(theta1, theta2, theta3)
        deltax = goalx - x
        deltay = goaly - y
        # print(theta1, theta2, theta3)
        # print(deltax, deltay)
        deltaT = [deltax, deltay]
        error1 = np.linalg.norm(deltaT)
        if error1 < error:
            print(x, y)
            # print("yes, We find angles")
            return theta1, theta2, theta3
        ja = jacbixy(theta1, theta2, theta3, deltax, deltay)
        deltaq = np.linalg.pinv(ja) @ deltaT
        # print(deltaq)
        theta1 = theta1 + step * deltaq[0]
        theta2 = theta2 + step * deltaq[1]
        theta3 = theta3 + step * deltaq[2]
        # theta1 = (theta1 + step*ja[0, 0]) % 360
        # theta2 = (theta2 + step*ja[1, 0]) % 360
        # theta3 = (theta3 + step*ja[2, 0]) % 360
        # print(theta1, theta2, theta3)
        iter = iter + 1
    return None


def Inverse(thetas, goalxy):
    thetas[1], thetas[2], thetas[3] = iter(thetas[1], thetas[2], thetas[3], goalxy[0], goalxy[1])
    while not ArmCheck(thetas[1], thetas[2], thetas[3]):
        thetas[1] = random.randrange(0, 90, 1)
        thetas[2] = random.randrange(0, 90, 1)
        thetas[3] = random.randrange(0, 90, 1)
        thetas[1], thetas[2], thetas[3] = iter(thetas[1], thetas[2], thetas[3], goalxy[0], goalxy[1])
    return thetas


if __name__ == '__main__':
    print(getxy(45, 45, 0))
    theta1, theta2, theta3 = iter(90, 0, 0, 1.707106781186548, 3.7071067811865475)
    print(ArmCheck(theta1, theta2, theta3))