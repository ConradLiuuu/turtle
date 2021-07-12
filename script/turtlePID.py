#! /usr/bin/env python

import rospy
import math
import cv2 as cv
import numpy as np

class Ctrler:
    def __init__(self):
        self.__PI = 3.14159
        
        self.x = 60
        self.y = 30
        self.fi = 0

        self.dst_x = 0
        self.dst_y = 0
        self.dst_fi = 0

        self.__delta_t = 0.001
        self.__theta = 0

        self.__lo = 0
        self.__alpha = 0
        self.__beta = 0

        self.__v = 0
        self.__w = 0

        self.fi = self.fi * self.__PI / 180 #degree to rad
        self.dst_fi = self.dst_fi * self.__PI / 180

        self.__k_lo = 1
        self.__k_alpha = 5
        self.__k_beta = -0.1

        self.__x_dot = 0
        self.__y_dot = 0
        self.__fi_dot = 0

        self.__img = np.zeros((512,512,3), np.uint8)
        
        self.__point_list = []

        self.__tolerance = 0.1

    def calculate_args(self):
        self.__theta = math.atan2((self.y-self.dst_y), (self.x-self.dst_x))
        
        self.__lo = math.sqrt(math.pow((self.x-self.dst_x), 2) + math.pow((self.y-self.dst_y), 2))
        self.__alpha = self.__theta + self.__PI - self.fi
        self.__beta = -(self.__alpha) - self.fi

        self.__v = self.__k_lo * self.__lo
        self.__w = self.__k_alpha * self.__alpha + self.__k_beta * self.__beta

        #calculate by turtle nethod
        #self.__w = 6 * (self.__theta - self.fi)

        #print("theta = %f" % self.__theta*180/self.__PI)
        #print("alpha = %f" % self.__alpha*180/self.__PI)
        #print("beta = %f" % self.__beta*180/self.__PI)

        print("v = %f" % self.__v)
        print("w = %f" % self.__w)

    def show_status(self):
        print("position = (%f, %f, %f)" % (self.x, self.y, self.fi*180/self.__PI))
        print("----------------------------------")

    def update_pose(self):
        self.x = self.x + math.cos(self.fi) * self.__v * self.__delta_t
        self.y = self.y + math.sin(self.fi) * self.__v * self.__delta_t
        self.fi = self.fi + self.__w * self.__delta_t

    def euclidean_distance(self):
        self.__lo = math.sqrt(math.pow((self.x-self.dst_x), 2) + math.pow((self.y-self.dst_y), 2))
        return self.__lo

    def linear_vel(self):
        self.__v =  1.5 * self.euclidean_distance()

    def steering_angle(self):
        #self.__theta = math.atan2((self.dst_y-self.y), (self.dst_x-self.x))
        self.__theta = math.atan2((self.y-self.dst_y), (self.x-self.dst_x))

    def angular_vel(self):
        #self.__fi = self.__fi + 
        self.__w = 6 * (self.__theta-self.dst_fi) 

    def arrive(self):
        if (abs(self.x-self.dst_x) < 0.02) and (abs(self.y-self.dst_y) < 0.02):
            return True
        else:
            return False

    def plot_coor(self):
        cv.rectangle(self.__img, (20,20), (480,480), (255,0,0), 5)
        cv.line(self.__img, (20,250), (480,250), (0,0,255), 1)
        cv.line(self.__img, (250,20), (250,480), (0,0,255), 1)
        #cv.circle(self.__img, (250,250), 1, (0,250,0), 7)
        cv.imshow("AAA", self.__img)
        cv.waitKey(0)

    def plot_path(self):
        #print("(%d,%d)" % (int(round(self.x+250)), int(round(-(self.y)+250))))
        self.__point_list.append((int(round(self.x+250)), int(round(-(self.y)+250))))
        for point in self.__point_list:
            cv.circle(self.__img, point, 3, (0,255,0), 1)
            cv.rectangle(self.__img, (int(round(self.x)+250-10), int(round(-(self.y))+250-20)), (int(round(self.x)+250+10), int(round(-(self.y))+250+20)),(255,0,0), 1)
        #cv.rectangle(self.__img, (20,20), (480,480), (255,0,0), 5)
        #cv.line(self.__img, (20,250), (480,250), (0,0,255), 1)
        #cv.line(self.__img, (250,20), (250,480), (0,0,255), 1)
        cv.imshow("AAA", self.__img)
        cv.waitKey(1)


if __name__ == '__main__':
    tt = Ctrler()
    tt.plot_coor()
    '''
    while tt.euclidean_distance() >= 0.1:
    #while (tt.arrive() != True):
        tt.update_pose()
        tt.plot_path()
        tt.calculate_args()
        tt.show_status()
    '''
    while tt.euclidean_distance() >= 0.1:
        tt.update_pose()
        tt.plot_path()
        tt.linear_vel()
        tt.steering_angle()
        tt.angular_vel()
        tt.show_status()
    
    tt.plot_coor()
    #return 0
