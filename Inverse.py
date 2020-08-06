from math import *
from Constant import *
import numpy as np

def getxy(theta1, theta2, theta3):
    theta1r = radians(theta1)
    theta2r = radians(theta2)
    theta3r = radians(theta3)
    x = l2*cos(theta1r) + l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r)
    y = l2*sin(theta1r) + l3*sin(theta1r+theta2r) + l4*sin(theta1r+theta2r+theta3r)
    return x, y


def jacbixy(theta1, theta2, theta3, deltax, deltay):
    theta1r = radians(theta1)
    theta2r = radians(theta2)
    theta3r = radians(theta3)
    j00 = - l2*sin(theta1r) -  l3*sin(theta1r+theta2r) - l4*sin(theta1r+theta2r+theta3r)
    j01 = -  l3*sin(theta1r+theta2r) - l4*sin(theta1r+theta2r+theta3r)
    j02 = - l4*sin(theta1r+theta2r+theta3r)
    j10 = l2*cos(theta1r) + l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r)
    j11 = l3*cos(theta1r+theta2r) + l4*cos(theta1r+theta2r+theta3r)
    j12 = l4*cos(theta1r+theta2r+theta3r)
    ja = np.zeros((3, 1))
    x = deltax / (abs(deltax) + abs(deltay))
    y = deltay / (abs(deltax) + abs(deltay))
    ja[0, 0] = j00*x + j10*y
    ja[1, 0] = j01*x + j11*y
    ja[2, 0] = j02*x + j12*y
    return ja

def iter(theta1, theta2, theta3, goalx, goaly):
    error = 0.0001
    iter = 0
    step = 0.5
    while iter < 10000:
        x,y = getxy(theta1, theta2, theta3)
        deltax = x - goalx
        deltay = y - goaly
        # print(deltax, deltay)
        if deltax<error and deltay<error:
            print(x, y)
            print(theta1, theta2, theta3)
            return theta1, theta2, theta3
        ja = jacbixy(theta1, theta2, theta3, deltax, deltay)
        theta1 = theta1 + step*ja[0, 0]
        theta2 = theta2 + step*ja[1, 0]
        theta3 = theta3 + step*ja[2, 0]




if __name__ == '__main__':
    # print(getxy(90, 0, 0))
    iter(0, 0, 0, 2, 2)