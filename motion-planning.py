import math as ma
import numpy as np
import Inverse as iv
import random

obstacles = [[[10, 10], [10, 20], [20, 20], [20, 10]]]


def GetUnsafeRange_v1(obstacles):
    # we only have the position coordinate
    ranges = []
    for obstacle in obstacles:
        angles = []
        for point in obstacle:
            angles.append(GetAngle(point))
        ranges.append([min(angles), max(angles)])
    print(ranges)


def GetAngle(coord):
    x = coord[0]
    y = coord[1]
    if x>=0 and y>=0:
        sin = y / np.sqrt(np.square(x)+np.square(y))
        angle = ma.degrees(ma.asin(sin))
        return angle


def RangeSafetyCheck(angle, obstacles):
    Unsafe_ranges = GetUnsafeRange_v1(obstacles)
    for range in Unsafe_ranges:
        if range[0] <= angle <= range[1]:
            return True
    return False



def MotionPlanning(obstacles, goal):
    thetas = [0, 90, 0, 0, 0, 0]
    goal1 = [goal[0], goal[1]]
    angle = GetAngle(goal1)
    if RangeSafetyCheck(angle, obstacles):
        return
    else:
        thetas[0] = angle
        xy = [np.squre(goal[0])+np.square(goal[1]), goal[2]]
        thetas[1], thetas[2], thetas[3] = iv.iter(thetas[1], thetas[2], thetas[3], xy[0], xy[1])
        while not iv.ArmCheck(thetas[1], thetas[2], thetas[3]):
            thetas[1] = random.randrange(0, 90, 1)
            thetas[2] = random.randrange(0, 90, 1)
            thetas[3] = random.randrange(0, 90, 1)
            thetas[1], thetas[2], thetas[3] = iv.iter(thetas[1], thetas[2], thetas[3], xy[0], xy[1])
            # int type notice
        return thetas


if __name__ == '__main__':
    GetUnsafeRange_v1(obstacles)