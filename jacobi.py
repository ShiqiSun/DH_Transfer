import numpy as np
import DH_tranfer as dh
import math as math


# def get_jac(thetas):
#     delta = 0.0001
#     jac = np.zeros((16, len(thetas)))
#     for i, joint in enumerate(thetas):
#         joints_m = list(thetas)
#         joints_p = list(thetas)
#         joints_m[i] -= delta
#         joints_p[p[i] += delta
#         Tm = dh.RobotArmModel_Feedforward(joints_m)
#         Tp = dh.RobotArmModel_Feedforward(joints_p)
#         jac[:, i] = (Tp - Tm).flatten() / (2 * delta)
#     return jac

def get_jac(joints: np.ndarray):
    delta = 0.0001
    jac = np.zeros((16, joints.shape[0]))
    for i, joint in enumerate(joints):
        joints_m = joints.copy()
        joints_p = joints.copy()
        joints_m[i] -= delta
        joints_p[i] += delta
        Tm = dh.RobotArmModel_Feedforward(joints_m)
        Tp = dh.RobotArmModel_Feedforward(joints_p)
        jac[:, i] = (Tp - Tm).flatten() / (2 * delta)
    return jac


def ik(T_tar, joints_init = np.zeros(6), tolerance = 1e-17):
    itertime = 0
    step = 0.5
    joints = joints_init.copy()
    while itertime < 100000:
        T_cur = dh.RobotArmModel_Feedforward(joints)
        deltaT = (T_tar - T_cur).flatten()
        error = np.linalg.norm(deltaT)
        if error < tolerance:
            return joints, radistransfer(joints)
        jac = get_jac(joints)
        deltaq = np.linalg.pinv(jac) @ deltaT
        joints = joints + step * deltaq
        itertime += 1
    return False


def radistransfer(joints):
    for i in range(len(joints)):
        joints[i] = np.round(math.degrees(joints[i])%360, 2)
        joints[i] = joints[i]%360
    return joints


def InverFind(base_coord, final_coord):
    # joints = ik(dh_table)
    return

