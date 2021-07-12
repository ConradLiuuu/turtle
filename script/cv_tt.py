#!/usr/bin/env python

import numpy as np
import cv2 as cv
import rospy
from std_msgs.msg import Float32MultiArray
import math

class Listener:
    def __init__(self):
        PI = 3.14159

        x = -3
        y = -4
        fi_ = 0
        dst_x = 0
        dst_y = 0
        dst_fi = 0
        theta = 0

        v = 0
        w = 0

        k_lo = 0.01
        k_alpha = 0.05
        k_beta = -0.0001

        x_dot = 0
        y_dot = 0
        fi_dot = 0
        
        lo_dot = 0
        alpha_dot = 0
        beta_dot = 0

        self.img = np.zeros((512,512,3), np.uint8)
        cv.rectangle(self.img, (20,20), (480,480), (255,0,0), 5)
        cv.circle(self.img, (250,250), 1, (255,0,0), 4)
        rospy.loginfo("show img")
        #cv.imshow("AAA", img)
        #cv.waitKey(1)

    def show_img(self):
        cv.imshow("AAA", self.img)
        cv.waitKey(1)

    def update_pos(self):
        x = x + x_dot
        y = y + y_dot
        fi_ = fi_ + fi_dot

        lo = lo + lo_dot
        alpha = alpha + alpha_dot
        beta = beta + beta_dot

    def arrive(self):
        if (math.abs(x-dst_x) < 0.5) and (math.abs(y-abs_y) < 0.5):
            return True
        else:
            return False

    def calculate_args(self):
        theta = math.atan2((y-dst_y), (x-dst_x)) * 180 / PI

        lo = math.sqrt(math.pow(x-dst_x, 2) + math.pow(y-dst_y, 2))
        alpha = theta + 180 - fi_
        beta = -(alpha) - fi_

        v = k_lo * lo
        w = k_alpha*alpha + k_beta*beta

        lo_dot = -(math.cos(alpha)) * v
        alpha_dot = math.sin((alpha/lo))*v - w
        beta_dot = -(math.sin((alpha/lo))) * v

        x_dot = math.cos(fi_) * v
        y_dot = math.sin(fi_) * v
        fi_dot = w

    def show_status(self):
        rospy.loginfo("theta = %f" % theta)
        rospy.loginfo("alpha = %f" % alpha)
        rospy.loginfo("beta = %f" % beta)

        rospy.loginfo("v = %f" % v)
        rospy.loginfo("w = %f" % w)

        rospy.loginfo("pos = %f, %f, %f" % (x, y, fi_))


'''
img = np.zeros((512, 512, 3), np.uint8)
#img.fill(200)

cv.rectangle(img, (20,20), (480, 480), (255, 0, 0), 5)
cv.circle(img, (250,250), 1, (255, 0, 0), 4)

cv.imshow("AAA", img)
cv.waitKey(0)
cv.destroyAllWindows()
'''


def listener():
    #rospy.init_node('cv_tt')
    rospy.loginfo('listener called')
    rospy.Subscriber("/turtle_pos", Float32MultiArray, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo('callback called')

    #rospy.loginfo(data.data[0]) 
    x = data.data[0] + 250
    y = -(data.data[1]) + 250
    #fi = data.data[2]

    point_list.append((int(round(x)), int(round(y))))
    #print(point_list)

    img = np.zeros((512,1024,3), np.uint8)
    cv.rectangle(img, (20,20), (480,480), (255,0,0), 3)
    cv.line(img, (20,250), (480,250), (0,0,255), 1)
    cv.line(img, (250,20), (250,480), (0,0,255), 1)

    #cv.circle(img, (int(x), int(y)), 1, (0,255,0), 4)
    for point in point_list:
        cv.circle(img, point, 1, (0,255,0), 5)
    cv.imshow("AAA", img)
    cv.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('cv_tt')
    #img = np.zeros((512,512,3), np.uint8)
    #cv.rectangle(img, (20,20), (480,480), (255,0,0), 5)
    #cv.circle(img, (250,250), 1, (255,0,0), 4) #origin point
    #cv.imshow("AAA", img)
    #cv.waitKey(10)
    #cv.circle(img, (258, 258), 1, (0,255,0), 4)
    #cv.imshow("AAA", img)
    #cv.waitKey(0)
    point_list = []

    listener()
    #rospy.spin()
