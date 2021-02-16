#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import time
from numpy.linalg import norm
from math import cos, sin, tan, sqrt, exp, pi, atan2, acos, asin
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


phi_imu, theta_imu, psi_imu = 0.0, 0.0, 0.0
height = height_des = 4.0

mp = [650, 483, height, 603, 476, height, 664, 49, height, 711, 55, 4.0]
mp_des = np.array([383.5, 480, height_des, 336.5, 480, height_des, 336.5, 0.0, height_des, 383.5, 0.0, height_des])


uav_vel_body = np.array([0.0, 0.0, 0.0, 0.0])
transCam = [0.0, 0.0, 0.14]
rotCam = [0.0, 1.57, 0.0]
phi_cam = rotCam[0]
theta_cam = rotCam[1]
psi_cam = rotCam[2]

# phi_cam, theta_cam, psi_cam = euler_from_quaternion (rotCam)
J1cam = [cos(theta_cam)*cos(psi_cam),  cos(psi_cam)*sin(theta_cam)*sin(phi_cam)-sin(psi_cam)*cos(phi_cam),  sin(psi_cam)*sin(phi_cam)+cos(psi_cam)*cos(phi_cam)*sin(theta_cam),
         cos(theta_cam)*sin(psi_cam),  cos(psi_cam)*cos(phi_cam)+sin(phi_cam)*sin(theta_cam) *
         sin(psi_cam),  sin(psi_cam)*sin(theta_cam) *
         cos(phi_cam)-cos(psi_cam)*sin(phi_cam),
         -sin(theta_cam),               cos(theta_cam)*sin(phi_cam),                                         cos(theta_cam)*cos(phi_cam)]

J1cam = np.array(J1cam).reshape(3, 3)

sst = np.array([[0.0, -transCam[2], transCam[1]],
                [transCam[2], 0.0, -transCam[0]],
                [-transCam[1], transCam[0], 0.0]]).reshape(3, 3)

Vcb = np.zeros((6, 6), dtype=float)
#print "J1cam:", J1cam
#print "Vcb:", Vcb
#print "Vcb[0:3, 0:3]:",  Vcb[0:3,0:3]
Vcb[0:3, 0:3] = J1cam
Vcb[3:6, 3:6] = J1cam
Vcb[0:3, 3:6] = np.dot(sst, J1cam)


Rphi = np.array([[1.0, 0.0, 0.0],[0.0, cos(phi_imu), -sin(phi_imu)],[0.0, sin(phi_imu), cos(phi_imu)]]).reshape(3,3)  
Rtheta = np.array([[cos(theta_imu), 0.0, sin(theta_imu)],[0.0, 1.0, 0.0],[-sin(theta_imu), 0.0, cos(theta_imu)]]).reshape(3,3)
Rft = np.dot(Rphi, Rtheta)

mpv0 = np.dot(Rft, mp[0:3])
mpv1 = np.dot(Rft, mp[3:6])
mpv2 = np.dot(Rft, mp[6:9])
mpv3 = np.dot(Rft, mp[9:12])
mpv = np.hstack((mpv0, mpv1, mpv2, mpv3))

cu = 360.5
cv = 240.5
ax = ay = 252.07 

x_0 = (mpv[0]-cu)/ax
y_0 = (mpv[1]-cv)/ay
Z_0 = mpv[2]

x_1 = (mpv[3]-cu)/ax
y_1 = (mpv[4]-cv)/ay
Z_1 = mpv[5]

x_2 = (mpv[6]-cu)/ax
y_2 = (mpv[7]-cv)/ay
Z_2 = mpv[8]

x_3 = (mpv[9]-cu)/ax
y_3 = (mpv[10]-cv)/ay
Z_3 = mpv[11]

xd_0 = (mp_des[0]-cu)/ax
yd_0 = (mp_des[1]-cv)/ay
Zd_0 = mp_des[2]

xd_1 = (mp_des[3]-cu)/ax
yd_1 = (mp_des[4]-cv)/ay
Zd_1 = mp_des[5]

xd_2 = (mp_des[6]-cu)/ax
yd_2 = (mp_des[7]-cv)/ay
Zd_2 = mp_des[8]

xd_3 = (mp_des[9]-cu)/ax
yd_3 = (mp_des[10]-cv)/ay
Zd_3 = mp_des[11]

Lm0 = np.array([[-1.0/Z_0, 0.0, x_0/Z_0, x_0*y_0, -(1.0+x_0*x_0), y_0],
                [0.0, -1.0/Z_0, y_0/Z_0, 1.0+y_0*y_0, -x_0*y_0, -x_0]]).reshape(2, 6)

Lm1 = np.array([[-1.0/Z_1, 0.0, x_1/Z_1, x_1*y_1, -(1.0+x_1*x_1), y_1],
                [0.0, -1.0/Z_1, y_1/Z_1, 1.0+y_1*y_1, -x_1*y_1, -x_1]]).reshape(2, 6)

Lm2 = np.array([[-1.0/Z_2, 0.0, x_2/Z_2, x_2*y_2, -(1.0+x_2*x_2), y_2],
                [0.0, -1.0/Z_2, y_2/Z_2, 1.0+y_2*y_2, -x_2*y_2, -x_2]]).reshape(2, 6)

Lm3 = np.array([[-1.0/Z_3, 0.0, x_3/Z_3, x_3*y_3, -(1.0+x_3*x_3), y_3],
                [0.0, -1.0/Z_3, y_3/Z_3, 1.0+y_3*y_3, -x_3*y_3, -x_3]]).reshape(2, 6)

Lm = np.concatenate((Lm0, Lm1, Lm2, Lm3), axis=0)
#print Lm
er_pix = 252.07*np.array([x_0-xd_0, y_0-yd_0, x_1-xd_1, y_1-yd_1, x_2 -
                          xd_2, y_2-yd_2, x_3-xd_3, y_3-yd_3]).reshape(8, 1)  # ax=ay=252.07

u = 252.07*np.array([x_0, x_1, x_2, x_3]).reshape(4, 1)
v = 252.07*np.array([y_0, y_1, y_2, y_3]).reshape(4, 1)
ud = 252.07*np.array([xd_0, xd_1, xd_2, xd_3]).reshape(4, 1)
vd = 252.07*np.array([yd_0, yd_1, yd_2, yd_3]).reshape(4, 1)


print("Errors: ", er_pix)
Kc = 0.01
Ucmd = -Kc*np.dot(np.linalg.pinv(Lm), er_pix)

UVScmd_uav_base_link = np.dot(Vcb, Ucmd)
print("UVScmd is: ", UVScmd_uav_base_link)
uav_vel_body[0] = UVScmd_uav_base_link[0]
print("1st uav vel body is: ", uav_vel_body[0])
uav_vel_body[1] = UVScmd_uav_base_link[1]
print("2nd uav vel body is: ", uav_vel_body[1])
uav_vel_body[2] = UVScmd_uav_base_link[2]
print("3rd uav vel body is: ", uav_vel_body[2])
uav_vel_body[3] = UVScmd_uav_base_link[5]
print("4th uav vel body is: ", uav_vel_body[3])