#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class Turtle:
    def __init__(self):
        rospy.init_node('PID', anonymous=False)
        self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subcriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.__lo = 0
        self.__theta = 0

        self.dst_fi = 0
        self.err_hist = 0
        self.err_t0 = 0

        self.show_theta = 0

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


    def euclidean_distance(self, goal_pose):
        self.__lo = sqrt(pow((goal_pose.x-self.pose.x), 2) + pow((goal_pose.y-self.pose.y), 2))
        return self.__lo

    def linear_vel(self, goal_pose, constant=0.75):
        #return constant * self.euclidean_distance(goal_pose) ## k_lo*lo

        err = self.euclidean_distance(goal_pose)
        kp = 0.75
        return kp*err

    def steering_angle(self, goal_pose):
        self.__theta = atan2((goal_pose.y-self.pose.y), (goal_pose.x-self.pose.x))
        #if self.__theta >= 2*3.14159:
            #self.__theta = self.__theta - 2*3.14159
        #elif self.__theta <= -(2*3.14159):
            #self.__theta = self.__theta + 2*3.14159

        return self.__theta

    def angular_vel(self, goal_pose, constant=6):
        #return constant * (self.steering_angle(goal_pose)-self.pose.theta)

        self.__alpha = self.steering_angle(goal_pose) + 3.14159 - self.pose.theta
        self.__beta = -(self.__alpha) - self.pose.theta

        #return 4*(5*self.__alpha + (-0.1)*self.__beta - 5*3.14159)

        k_alpha = 5
        k_beta = -(0.1)
        err = k_alpha*self.__alpha + k_beta*self.__beta - k_alpha*3.14159
        self.err_hist += err
        diff = err - self.err_t0
        self.err_t0 = err

        kp = 2
        ki = 0.002
        kd = 1

        #return kp*err + ki*self.err_hist + kd*diff
        ## 2 0.002 1
        return kp*err ##kp=3
        #return kp*err + kd*diff

    def rotate_itself(self):
        if self.pose.theta >= 2*3.14159:
            self.pose.theta = self.pose.theta - 2*3.14159
        elif self.pose.theta <= -(2*3.14159):
            self.pose.theta = self.pose.theta + 2*3.14159

        vel_msg = Twist()
        self.__goal_pose.theta = self.__goal_pose.theta

        #while (abs(self.pose.theta) - abs(self.__goal_pose.theta)) >= 0.5 * 0.01745:
        fine = 5
        while round(self.pose.theta,fine) != round(self.__goal_pose.theta,fine):
            err = round(self.__goal_pose.theta, fine) - round(self.pose.theta, fine)
            diff = err - self.err_t0
            self.err_hist += err

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 2 * err
            #vel_msg.angular.z = 0.2 * err + 0.001*self.err_hist + 0.1*diff
            ##origin kp = 0.5

            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()
        #print(self.pose.theta * 180 / 3.14159)
        #self.show_pose()

    def show_pose(self):
        print("destination pose: x = %f, y = %f, theta = %f" % (self.pose.x, self.pose.y, self.pose.theta*180/3.14159))

    def conti_path(self, p):
        goal_pose = Pose()
        goal_pose.x = p[0]
        goal_pose.y = p[1]

        self.__goal_pose = Pose()
        self.__goal_pose.x = goal_pose.x
        self.__goal_pose.y = goal_pose.y

        distance_tolerance = 0.1

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        self.rotate_itself(p[2])

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = input("Set x goal:")
        goal_pose.y = input("Set y goal:")
        goal_pose.theta = input("Set theta goal:")

        if goal_pose.theta >= 180:
            goal_pose.theta = 180 - goal_pose.theta
        goal_pose.theta = goal_pose.theta*3.14159/180

        self.__goal_pose = Pose()
        self.__goal_pose.x = goal_pose.x
        self.__goal_pose.y = goal_pose.y
        self.__goal_pose.theta = goal_pose.theta

        distance_tolerance = input("Set tolerance:")

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        
        #rospy.spin()

if __name__ == '__main__':
    '''
    try:
        tt = Turtle()
        tt.move2goal()
    except rospy.ROSInterruptException:
        pass
    '''
    #pp = [(2,2), (2,8), (8,8), (8,2), (2,2)]
    #pp = [(8,2,-180), (2,2,90), (2,8,0), (8,8,-90), (8,2,-180)]
    tt = Turtle()
    tt.move2goal()
    tt.rotate_itself()
    tt.show_pose()

    #for point in pp:
        #tt.conti_path(point)
