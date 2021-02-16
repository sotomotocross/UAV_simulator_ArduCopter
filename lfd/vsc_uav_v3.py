#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('lfd')
import sys
import rospy
import cv2
import numpy as np
import time
import json
import os
import matplotlib.pyplot as plt
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import Empty, Int16, Float32, Bool, UInt16MultiArray, UInt32MultiArray, UInt64MultiArray
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3Stamped, TwistStamped
from numpy.linalg import norm
from math import cos, sin, tan, sqrt, exp, pi, atan2, acos, asin
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from lfd.msg import VSCdata
from operator import itemgetter



class image_converter:
  
    def __init__(self):
        #Create publishers
        self.pub_vel = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.pub_error = rospy.Publisher('error', Int16, queue_size=10)
        self.pub_angle = rospy.Publisher('angle', Int16, queue_size=10)
        self.pub_vsc_data = rospy.Publisher("/vsc_data", VSCdata, queue_size=1000)
        self.pub_im = rospy.Publisher('im', Image, queue_size=10)
        self.bridge = CvBridge()
        #Create subscribers
        self.image_sub = rospy.Subscriber("/image_raww", Image, self.callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.updateImu)
        self.pos_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.OdomCb)
        self.vel_uav = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.VelCallback)
        
        # uav state variables
        self.landed = 0
        self.takeoffed = 1
        self.uav_vel_body = np.array([0.0, 0.0, 0.0, 0.0])
        self.vel_uav = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.er_pix_prev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # ZED stereo camera translation and rotation variables
        self.transCam = [0.0, 0.0, 0.14]
        self.rotCam = [0.0, -1.57, 0.0]
        self.phi_cam = self.rotCam[0]
        self.theta_cam = self.rotCam[1]
        self.psi_cam = self.rotCam[2]
        # ZED stereocamera intrinsic parameters
        self.cu = 360.5
        self.cv = 240.5
        self.ax = 252.07
        self.ay = 252.07
        # Variables initialization
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.phi_imu = 0.0
        self.theta_imu = 0.0 
        self.psi = 0.0
        self.t = 0.0
        self.dt = 0.03
        self.a = 0.2


    def calculateJ1(self, phi, theta, psi):
        
        J1 = [cos(theta)*cos(psi),  cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta),
            cos(theta)*sin(psi),  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi),  sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),
            -sin(theta),          cos(theta)*sin(phi),                             cos(theta)*cos(phi)]
        J1 = np.array(J1).reshape(3,3)
        
        return J1
    
    
    def updateImu(self, msg):
        self.phi_imu = msg.orientation.x
        self.theta_imu = msg.orientation.y
        self.psi_imu = msg.orientation.z
        self.w_imu = msg.orientation.w
        self.phi_imu, self.theta_imu, self.psi_imu = euler_from_quaternion ([self.phi_imu, self.theta_imu, self.psi_imu, self.w_imu])
    
    
    def OdomCb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        # print("Message position: ", msg.pose.pose.position)

    def VelCallback(self, msg):
    
        self.vel_uav[0] = msg.twist.linear.x
        self.vel_uav[1] = msg.twist.linear.y
        self.vel_uav[2] = msg.twist.linear.z
        self.vel_uav[3] = msg.twist.angular.x
        self.vel_uav[4] = msg.twist.angular.y
        self.vel_uav[5] = msg.twist.angular.z
        # print("Message uav velocity: ", self.vel_uav)
    
    
    def featuresTransformation(self, mp, phi, theta):
        
        Rphi = np.array([[1.0, 0.0, 0.0],[0.0, cos(phi), -sin(phi)],[0.0, sin(phi), cos(phi)]]).reshape(3,3)
        Rtheta = np.array([[cos(theta), 0.0, sin(theta)],[0.0, 1.0, 0.0],[-sin(theta), 0.0, cos(theta)]]).reshape(3,3)
        Rft = np.dot(Rphi, Rtheta)
        mpv0 = np.dot(Rft, mp[0:3])
        mpv1 = np.dot(Rft, mp[3:6])
        mpv2 = np.dot(Rft, mp[6:9])
        mpv3 = np.dot(Rft, mp[9:12])
        mpv = np.hstack((mpv0, mpv1, mpv2, mpv3))
        
        return mpv
    
    
    def calculateIM(self, mpv, mp_des, cu, cv, ax, ay):
        
        x_0 = (mpv[0]-cu)/ax
        y_0 = (mpv[1]-cv)/ay
        Z_0 = self.z

        x_1 = (mpv[3]-cu)/ax
        y_1 = (mpv[4]-cv)/ay
        Z_1 = self.z
        
        x_2 = (mpv[6]-cu)/ax
        y_2 = (mpv[7]-cv)/ay
        Z_2 = self.z
            
        x_3 = (mpv[9]-cu)/ax
        y_3 = (mpv[10]-cv)/ay
        Z_3 = self.z

        xd_0 = (mp_des[0]-cu)/ax
        yd_0 = (mp_des[1]-cv)/ay
        Zd_0 = self.z

        xd_1 = (mp_des[3]-cu)/ax
        yd_1 = (mp_des[4]-cv)/ay
        Zd_1 = self.z

        xd_2 = (mp_des[6]-cu)/ax
        yd_2 = (mp_des[7]-cv)/ay
        Zd_2 = self.z

        xd_3 = (mp_des[9]-cu)/ax
        yd_3 = (mp_des[10]-cv)/ay
        Zd_3 = self.z
                    
        Lm0 = np.array([[-1.0/Z_0, 0.0, x_0/Z_0, x_0*y_0, -(1.0+x_0*x_0), y_0],
                        [0.0, -1.0/Z_0, y_0/Z_0, 1.0+y_0*y_0, -x_0*y_0, -x_0]]).reshape(2,6)
        Lm1 = np.array([[-1.0/Z_1, 0.0, x_1/Z_1, x_1*y_1, -(1.0+x_1*x_1), y_1],
                        [0.0, -1.0/Z_1, y_1/Z_1, 1.0+y_1*y_1, -x_1*y_1, -x_1]]).reshape(2,6)
        Lm2 = np.array([[-1.0/Z_2, 0.0, x_2/Z_2, x_2*y_2, -(1.0+x_2*x_2), y_2],
                        [0.0, -1.0/Z_2, y_2/Z_2, 1.0+y_2*y_2, -x_2*y_2, -x_2]]).reshape(2,6)
        Lm3 = np.array([[-1.0/Z_3, 0.0, x_3/Z_3, x_3*y_3, -(1.0+x_3*x_3), y_3],
                        [0.0, -1.0/Z_3, y_3/Z_3, 1.0+y_3*y_3, -x_3*y_3, -x_3]]).reshape(2,6)
        Lm = np.concatenate((Lm0, Lm1, Lm2, Lm3), axis=0)
        er_pix = np.array([x_0-xd_0, y_0-yd_0, x_1-xd_1, y_1-yd_1, x_2-xd_2, y_2-yd_2, x_3-xd_3, y_3-yd_3 ]).reshape(8,1) #ax=ay=252.07
        
        return Lm, er_pix

    def quadrotorVSControl(self, Lm, er_pix):
        
        Kc = 0.8
        Ucmd = -np.dot(Kc,np.dot(np.linalg.pinv(Lm), er_pix))+np.dot(np.linalg.pinv(Lm), np.array([0.0, 0.0, 0.0, 0.0, 0.0, self.a, 0.0, self.a]).reshape(8,1))
        
        return Ucmd
        
    def quadrotorVSControl_tracking(self, Lm, er_pix, vel_camera, er_pix_prev):
        
        # Kc = 1.2
        first_gain_Kc = 0.8
        yaw_gain_Kc = 2.5
        Kc = np.identity(6)
        Kc[0][0] = first_gain_Kc
        Kc[1][1] = first_gain_Kc
        Kc[2][2] = first_gain_Kc
        Kc[3][3] = 0.0
        Kc[4][4] = 0.0
        Kc[5][5] = yaw_gain_Kc

        Ke = 0.06
        # Ke = np.identity(6)
        # first_gain_Ke = 0.001
        # yaw_gain_Ke = 0.003
        # Kc = np.identity(6)
        # Kc[0][0] = first_gain_Ke
        # Kc[1][1] = first_gain_Ke
        # Kc[2][2] = first_gain_Ke
        # Kc[3][3] = 0.0
        # Kc[4][4] = 0.0
        # Kc[5][5] = yaw_gain_Ke

        ew = (er_pix - er_pix_prev)/self.dt - np.array(np.dot(Lm, vel_camera)).reshape(8,1)  
        # print("ew: ", np.dot(np.linalg.pinv(Lm), ew))    
        # Ucmd = -np.dot(Kc,np.dot(np.linalg.pinv(Lm), er_pix))+np.dot(np.linalg.pinv(Lm), np.array([0.0, 0.0, 0.0, 0.0, 0.0, self.a, 0.0, self.a]).reshape(8,1)) - np.dot(Ke, np.dot(np.linalg.pinv(Lm), ew))
        Ucmd = -np.dot(Kc,np.dot(np.linalg.pinv(Lm), er_pix))+np.dot(np.linalg.pinv(Lm), np.array([0.0, 0.0, 0.0, 0.0, 0.0, self.a, 0.0, self.a]).reshape(8,1)) - Ke*np.dot(np.linalg.pinv(Lm), ew)
        
        return Ucmd
    
    # Detect the line and piloting
    def line_detect(self, cv_image):
        # Create a mask
        # cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(cv_image, (130, 130, 130), (255, 255, 255))
        kernel = np.ones((1, 1), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=3)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # contours_blk, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk.sort(key=cv2.minAreaRect)
        
        # Features vectors initialization
        mp_v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mp_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        if len(contours_blk) > 0 and cv2.contourArea(contours_blk[0]) > 200:
            # Box creation for the detected coastline
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, y_min), (w_min, h_min), angle = blackbox            
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)

            # Sorting of the orientation of the detected coastline
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle

            if angle >= 0:
                print("1st control choice")
                print("Angle:", angle)
                mp = [box[0][0], box[0][1], self.z, box[1][0], box[1][1], self.z, box[2][0], box[2][1], self.z, box[3][0], box[3][1], self.z]
                # print("mp: ", mp)
            else:
                print("2nd control choice")
                print("Angle:", angle)
                mp = [box[3][0], box[3][1], self.z, box[0][0], box[0][1], self.z, box[1][0], box[1][1], self.z, box[2][0], box[2][1], self.z]
                # print("mp: ", mp)    

            # Features transformation and desired features definition
            mp_v = self.featuresTransformation(mp, self.phi_imu, self.theta_imu) #TRANSFORM FEATURES
            # print("mp_v: ", mp_v)    
            mp_des = np.array([420, 472, self.z, 367, 483, self.z, 327, 2+self.a*self.t, self.z, 377, 0+self.a*self.t, self.z])            
            # print("mp_des: ", mp_des)            
            
            cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 1)
            cv2.line(cv_image, (int(x_min), 54), (int(x_min), 74), (255, 0, 0), 1)
            
            # Velocity transformation from the camera to the base_link frame
            J1cam = self.calculateJ1(self.phi_cam, self.theta_cam, self.psi_cam) # monadiaios
            sst = np.array([[0.0, -self.transCam[2], self.transCam[1]],
                             [self.transCam[2], 0.0, -self.transCam[0]],
                             [-self.transCam[1], self.transCam[0], 0.0]]).reshape(3,3)
            Vcb = np.zeros((6,6), dtype = float)
            Vcb[0:3, 0:3] = J1cam
            Vcb[3:6, 3:6] = J1cam
            Vcb[0:3, 3:6] = np.dot(sst,J1cam)

            # Interaction matrix, error of pixels and velocity commands calculation (a.k.a control execution)
            Lm, er_pix = self.calculateIM(mp_v, mp_des, self.cu, self.cv, self.ax, self.ay) #TRANSFORM FEATURES
            # print("Error pixel: ", er_pix)     
            velocity_camera = np.dot(np.linalg.inv(Vcb), self.vel_uav)
            # print("Camera velocity: ", velocity_camera)
            # UVScmd = self.quadrotorVSControl(Lm, er_pix)
            UVScmd = self.quadrotorVSControl_tracking(Lm, er_pix, velocity_camera, self.er_pix_prev)
            UVScmd = np.dot(Vcb, UVScmd)
            self.er_pix_prev = er_pix
            # print("Previous error: ", self.er_pix_prev)
            # print("UVScmd is: ", UVScmd)
             
            self.uav_vel_body[0] = UVScmd[0]
            self.uav_vel_body[1] = UVScmd[1]
            self.uav_vel_body[2] = UVScmd[2]
            self.uav_vel_body[3] = UVScmd[5]
            # print("1st uav vel body is: ", self.uav_vel_body[0])
            # print("2nd uav vel body is: ", self.uav_vel_body[1])
            # print("3rd uav vel body is: ", self.uav_vel_body[2])
            # print("4th uav vel body is: ", self.uav_vel_body[3])
            
            twist = PositionTarget()
            #twist.header.stamp = 1
            twist.header.frame_id = 'world'
            twist.coordinate_frame = 8
            twist.type_mask = 1479
            twist.velocity.x = self.uav_vel_body[0]
            twist.velocity.y = self.uav_vel_body[1]
            twist.velocity.z = self.uav_vel_body[2]
            twist.yaw_rate = self.uav_vel_body[3]
            # twist.velocity.x = 0.0
            # twist.velocity.y = 0.0
            twist.velocity.z = 0.0
            # twist.yaw_rate = 0.0

            vsc_msg = VSCdata()
            vsc_msg.errors = er_pix
            vsc_msg.cmds = self.uav_vel_body
            self.pub_vsc_data.publish(vsc_msg)
            self.pub_vel.publish(twist)      
            
        ros_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub_im.publish(ros_msg)

        # if len(contours_blk) == 0 and self.was_line == 1 and self.line_back == 1:
        #     twist = PositionTarget()
        #     if self.line_side == -1:  # line at the right
        #         twist.header.frame_id = 'world'
        #         twist.coordinate_frame = 8
        #         twist.type_mask = 1479
        #         #twist.velocity.x = 0
        #         twist.velocity.y = -0.1
        #         #twist.velocity.z = 0
        #         #twist.yaw_rate = 0
        #         self.pub_vel.publish(twist)
        #     if self.line_side == 1:  # line at the left
        #         twist.header.frame_id = 'world'
        #         twist.coordinate_frame = 8
        #         twist.type_mask = 1479
        #         #twist.velocity.x = 0
        #         twist.velocity.y = 0.1
        #         #twist.velocity.z = 0
        #         #twist.yaw_rate = 0
        #         self.pub_vel.publish(twist)
        
        self.t = self.t+self.dt
  
  
  # Image processing @ 10 FPS
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        if self.takeoffed and (not self.landed):
            self.line_detect(cv_image)
            cv_image = cv2.resize(cv_image, (720, 480))
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1) & 0xFF

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    # time.sleep(0.03)
    rospy.sleep(0.03)
    #ic.cam_down()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)