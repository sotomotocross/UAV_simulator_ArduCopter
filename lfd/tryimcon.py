#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('lfd')
import sys
import rospy
import cv2
import numpy as np
import time
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Twist


#from geometry_msgs.msg import Point
#from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from std_msgs.msg import Int16  # For error/angle plot publishing
from sensor_msgs.msg import Image
#from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

    def __init__(self):
        self.pub_vel = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        # self.pub_vel = rospy.Publisher('/mavros/setpoint_vel/cmd_vel', PositionTarget, queue_size=1)
        self.pub_error = rospy.Publisher('error', Int16, queue_size=10)
        self.pub_angle = rospy.Publisher('angle', Int16, queue_size=10)
        self.pub_im = rospy.Publisher('im', Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raww", Image, self.callback)
        
        # self.Kp = 0.112                 # Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
        # self.Ki = 0
        # self.kd = 0.03
        self.Kp = 0.2                 # Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
        self.Ki = 0
        self.kd = 0.8
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        # self.Kp_ang = 0.015             # Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
        # self.Ki_ang = 0
        # self.kd_ang = 0.015
        self.Kp_ang = 0.01             # Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
        self.Ki_ang = 0
        self.kd_ang = 0.008
        self.integral_ang = 0
        self.derivative_ang = 0
        self.last_ang = 0
        self.was_line = 0
        self.line_side = 0
        self.battery = 0
        self.line_back = 1
        self.landed = 0
        self.takeoffed = 1
        self.error = []
        self.angle = []
        self.fly_time = 0.0
        self.start = 0.0
        self.stop = 0.0
        self.velocity = 8.0 #0.06


    # Detect the line and piloting
    def line_detect(self, cv_image):
        # Create a mask
        # cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(cv_image, (130, 130, 130), (255, 255, 255))
        kernel = np.ones((1, 1), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=3)
        mask = cv2.dilate(mask, kernel, iterations=3)
        contours_blk, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # _, contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       
        contours_blk.sort(key=cv2.minAreaRect)
        
        if len(contours_blk) > 0 and cv2.contourArea(contours_blk[0]) > 200:
            self.was_line = 1
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, y_min), (w_min, h_min), angle = blackbox        
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle
            #angle = 0
            setpoint = cv_image.shape[1] / 2
            error = int(x_min - setpoint)
            #error = 20
            self.error.append(error)
            self.angle.append(angle)
            normal_error = float(error) / setpoint

            if error > 0:
                self.line_side = -1  # line in right
            elif error <= 0:
                self.line_side = 1  # line in left

            self.integral = float(self.integral + normal_error)
            self.derivative = normal_error - self.last_error
            self.last_error = normal_error        


            error_corr = 1*(self.Kp * normal_error + self.Ki * self.integral + self.kd * self.derivative)  # PID controler
            #kanonika me -1* print("error_corr:  ", error_corr, "\nP", normal_error * self.Kp, "\nI", self.integral* self.Ki, "\nD", self.kd * self.derivative)

            angle = int(angle)
            normal_ang = float(angle) / 90

            # e.write("%d\n" %normal_error)
            # a.write("%d\n" %normal_ang)


            self.integral_ang = float(self.integral_ang + angle)
            self.derivative_ang = angle - self.last_ang
            self.last_ang = angle

            ang_corr = -1 * (self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler            
         

            box = cv2.boxPoints(blackbox)
            box = np.int0(box)

            cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 1)

            cv2.putText(cv_image, "Angle: " + str(angle), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 2,
                        cv2.LINE_AA)

            cv2.putText(cv_image, "Error: " + str(error), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2,
                        cv2.LINE_AA)
            cv2.line(cv_image, (int(x_min), 54), (int(x_min), 74), (255, 0, 0), 1)


            twist = PositionTarget()
            #twist.header.stamp = 1
            twist.header.frame_id = 'world'
            twist.coordinate_frame = 8
            twist.type_mask = 1479
            twist.velocity.x = self.velocity
            twist.velocity.y = error_corr
            #twist.velocity.z = 0
            twist.yaw_rate = ang_corr

            #twist.position.z = ang_corr
            self.pub_vel.publish(twist)
            # print("angVal: ", twist.angular.z)

            ang = Int16()
            ang.data = angle
            self.pub_angle.publish(ang)

            err = Int16()
            err.data = error
            self.pub_error.publish(err)

	    ros_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
	    self.pub_im.publish(ros_msg)

        if len(contours_blk) == 0 and self.was_line == 1 and self.line_back == 1:
            twist = PositionTarget()
            if self.line_side == -1:  # line at the right
                twist.header.frame_id = 'world'
                twist.coordinate_frame = 8
                twist.type_mask = 1479
                #twist.velocity.x = 0
                twist.velocity.y = -1.0
                #twist.velocity.z = 0
                #twist.yaw_rate = 0
                self.pub_vel.publish(twist)
            if self.line_side == 1:  # line at the left
                twist.header.frame_id = 'world'
                twist.coordinate_frame = 8
                twist.type_mask = 1479
                #twist.velocity.x = 0
                twist.velocity.y = 1.0
                #twist.velocity.z = 0
                #twist.yaw_rate = 0
                self.pub_vel.publish(twist)
	
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1) & 0xFF

    
    def errorPlot(self):
        meanError = np.mean(self.error)
        stdError = np.std(self.error)
        meanAngle = np.mean(self.angle)
        stdAngle = np.std(self.angle)
        self.fly_time = self.stop - self.start
        print("""
      /*---------------------------------------------
              meanError: %f[px],   stdError: %f[px]
              meanAngle: %f[deg],   stdAngle: %f[deg]
              Time: %f[sec], Velocity: %f[percent]
      ---------------------------------------------*/
      """ %(meanError, stdError, meanAngle, stdAngle, self.fly_time, self.velocity))


    # Image processing @ 10 FPS
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.takeoffed and (not self.landed):
            self.line_detect(cv_image)
            #self.land_detect(cv_image)
        cv2.putText(cv_image, "battery: " + str(self.battery) + "%", (570, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 0), 2, cv2.LINE_AA)
	#cv_image = cv2.resize(cv_image, (480, 480))
        cv2.imshow("Image window", cv_image)
        # cv2.imshow("mask", mask)
        cv2.waitKey(1) & 0xFF
	

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    time.sleep(3)
    #ic.cam_down()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
