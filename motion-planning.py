import math as ma
import numpy as np
import Inverse as iv
import random
import rrt_2d as rrt
from Constant import *
import envplot as plt


def FindInter(point1, point2, angle):
    angle = ma.radians(angle)
    k = ma.tan(angle)
    A = point2[1] - point1[1]
    B = point1[0] - point2[0]
    C = point2[0]*point1[1] - point1[0]*point2[1]
    x = -1*C / (A+B*k)
    y = x*k
    return [x, y]


def LineObstacle(angle, obstacles, heights):
    interobs = []
    flag = 0
    for index1, obstacle in enumerate(obstacles):
        points = []
        for index in range(len(obstacle)):
            angle1 = GetAngle(obstacle[index-1])
            angle2 = GetAngle(obstacle[index])
            if min(angle1, angle2) < angle <= max(angle1, angle2):
                a = FindInter(obstacle[index-1], obstacle[index], angle)
                a.append(heights[index1])
                points.append(a)
        if len(points) > 0:
            interobs.append(points)
            # print(angle1, angle2)
    return interobs


def GetUnsafeRange_v1(obstacles):
    # we only have the position coordinate
    ranges = []
    for obstacle in obstacles:
        angles = []
        for point in obstacle:
            angles.append(GetAngle(point))
        ranges.append([min(angles), max(angles)])
    # print(ranges)
    return ranges



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


def MotionPlanning(obstacles, goal, heights):
    thetas = [0, 90, 0, 0, 90, 0]
    goal1 = [goal[0], goal[1]]
    plt.obplot(obstacles, goal1)
    angle = GetAngle(goal1)
    goalxy = [np.sqrt(np.square(goal[0]) + np.square(goal[1])), goal[2]]
    if RangeSafetyCheck(angle, obstacles):
        thetas[0] = angle
        # RRTMotionPlanning()
        interobs = LineObstacle(angle, obstacles, heights)
        # upbound = 0
        # lowbound = 0
        new_obstacles = []
        for ob in interobs:
            d = []
            h = ob[0][2]
            for point in ob:
                d.append(np.sqrt(np.square(point[0])+np.square(point[1])))
            upbound = max(d)
            lowbound = min(d)
            obstacle = [lowbound, upbound, h]
            new_obstacles.append(obstacle)
        while not rrt.RRT_arm(new_obstacles, goalxy, thetas):
            continue
        return True
    else:
        thetas[0] = angle
        thetas[1], thetas[2], thetas[3] = iv.iter(thetas[1], thetas[2], thetas[3], goalxy[0], goalxy[1])
        while not iv.ArmCheck(thetas[1], thetas[2], thetas[3]):
            thetas[1] = random.randrange(0, 90, 1)
            thetas[2] = random.randrange(0, 90, 1)
            thetas[3] = random.randrange(0, 90, 1)
            thetas[1], thetas[2], thetas[3] = iv.iter(thetas[1], thetas[2], thetas[3], goalxy[0], goalxy[1])
            # int type notice
        print("There is no obstacles in our direction. Directly go and grap it.")
        print("The Angles is:", thetas)
        return True


if __name__ == '__main__':
    # GetUnsafeRange_v1(obstacles)
    MotionPlanning(obstacles, [10, 10, 5], heights)
    # print(LineObstacle(45, obstacles, heights))