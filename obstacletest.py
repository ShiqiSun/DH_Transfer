from Constant import *
import math

Joint1 = [0, l1]


def getJoint(thetas, joint):
    theta1r = math.radians(thetas[1])
    theta2r = math.radians(thetas[2])
    theta3r = math.radians(thetas[3])
    if joint <= 0 or joint > 5:
        print("Wrong Joint")
        exit(1)
        return [-1, -1]

    switcher = {

        1:[0, l1],

        2:[l2 * math.cos(theta1r), l1 + l2 * math.sin(theta1r)],

        3:[l2 * math.cos(theta1r) + l3*math.cos(theta1r+theta2r),
           l1 + l2 * math.sin(theta1r) + l3*math.sin(theta1r+theta2r)],

        4:[l2 * math.cos(theta1r) + l3*math.cos(theta1r+theta2r)
           + l4*math.cos(theta1r+theta2r+theta3r),
           l1 + l2 * math.sin(theta1r) + l3*math.sin(theta1r+theta2r)
           + l4*math.sin(theta1r+theta2r+theta3r)],

        5:[l2 * math.cos(theta1r) + l3*math.cos(theta1r+theta2r)
           + l4*math.cos(theta1r+theta2r+theta3r) + l5*math.sin(theta1r+theta2r+theta3r),
           l1 + l2 * math.sin(theta1r) + l3 * math.sin(theta1r + theta2r)
           + l4 * math.sin(theta1r + theta2r + theta3r)+ - l5*math.cos(theta1r+theta2r+theta3r)],

    }
    return switcher[joint]


def cross(p1,p2,p3): # 叉积判定
    x1=p2[0]-p1[0]
    y1=p2[1]-p1[1]
    x2=p3[0]-p1[0]
    y2=p3[1]-p1[1]
    return x1*y2-x2*y1


def segment(p1,p2,p3,p4):
    if(max(p1[0],p2[0]) >= min(p3[0],p4[0])
    and max(p3[0],p4[0]) >= min(p1[0],p2[0])
    and max(p1[1],p2[1]) >= min(p3[1],p4[1])
    and max(p3[1],p4[1]) >= min(p1[1],p2[1])):
      if(cross(p1 , p2, p3)*cross(p1,p2,p4)<=0
        and cross(p3,p4,p1)*cross(p3,p4,p2)<=0):
        D = True
      else:
        D = True
    else:
      D = False
    return D


def LineObcheck(l1, l2, sq):
  if (sq[0] <= l1[0] <= sq[2] and sq[1] <= l1[1] <= sq[3]) \
          or (sq[0] <= l2[0] <= sq[2] and sq[1] <= l2[1] <= sq[3]):
    return True

  else:
    # step 2 check if diagonal cross the segment
    p1 = [sq[0], sq[1]]
    p2 = [sq[2], sq[3]]
    p3 = [sq[2], sq[1]]
    p4 = [sq[0], sq[3]]
    if segment(l1, l2, p1, p2) or segment(l1, l2, p3, p4):
      return True
    else:
      return False


def ArmStatusCheck(thetas, ob):
    for i in range(4):
        point1 = getJoint(thetas, i + 1)
        point2 = getJoint(thetas, i + 2)
        if LineObcheck(point1, point2, ob):
            print("This Path is Wrong")
            return True
    return False


if __name__ == '__main__':
    thetas = [0, 90, 0, 0, 90, 0]
    # ob = [0, 0, 1, 1]
    # print(LineObcheck([-1, 0], [-2, -3], ob))
    # ArmStatusCheck(thetas)