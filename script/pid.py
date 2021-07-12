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
        self.interr = 0
        self.histerr = 0

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


    def euclidean_distance(self, goal_pose):
        self.__lo = sqrt(pow((goal_pose.x-self.pose.x), 2) + pow((goal_pose.y-self.pose.y), 2))
        return self.__lo

    def linear_vel(self, goal_pose, constant=0.75):
        return constant * self.euclidean_distance(goal_pose) ## k_lo*lo

    def steering_angle(self, goal_pose):
        self.__theta = atan2((goal_pose.y-self.pose.y), (goal_pose.x-self.pose.x))
        return self.__theta

    def angular_vel(self, goal_pose, constant=6):
        #return constant * (self.steering_angle(goal_pose)-self.pose.theta)

        self.__alpha = self.steering_angle(goal_pose) + 3.14159 - self.pose.theta
        self.__beta = -(self.__alpha) - self.pose.theta

        return 5*self.__alpha + (-0.1)*self.__beta - 5*3.14159

    def rotate_ctrl(self, p):
        print("call rot")
        vel_msg = Twist()
        angle = p
        angle = angle * 3.14159 / 180

        #print(abs(angle) - abs(self.pose.theta))

        while (abs(abs(angle) - abs(self.pose.theta))) >= 0.5 * 0.01745:
            err = round(angle, 4) - round(self.pose.theta, 4)
            self.interr += err
            diff = err - self.histerr
            self.histerr = err

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 1.5 * err + 0.01 * self.interr + 1 * diff
            print("degree = %f" % (1.5*err + 0.01*self.interr + 1*diff))

            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()
        
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)

    def linear_ctrl(self, point):
        #print("called linear")
        goal_pose = Pose()
        goal_pose.x = point[0]
        goal_pose.y = point[1]

        distance_tolerance = 0.1

        vel_msg = Twist()

        #print(abs(goal_pose.x-self.pose.x), abs(goal_pose.y-self.pose.y))

        #while self.euclidean_distance(goal_pose) >= distance_tolerance:
        while abs(goal_pose.x-self.pose.x) >= 0.1 or abs(goal_pose.y-self.pose.y) >= 0.1:
        #while (pow((goal_pose.x-self.pose.x), 2))+(pow((goal_pose.y-self.pose.y), 2)) > distance_tolerance:
            err = (pow((goal_pose.x-self.pose.x), 2))+(pow((goal_pose.y-self.pose.y), 2))
            #err = self.euclidean_distance(goal_pose)
            #if atan2(goal_pose.y-self.pose.y, goal_pose.x-self.pose.x) <= 0:
                #err = err
            #else:
                #err = -(err)
            #print(err)
            #print(goal_pose.x, goal_pose.y)

            vel_msg.linear.x = 0.5 * err
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        #vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        self.rate.sleep()
        self.rotate_ctrl(point[2])

    def rotate_itself(self):
        if self.pose.theta >= 2*3.14159:
            self.pose.theta = self.pose.theta - 2*3.14159
        elif self.pose.theta <= -(2*3.14159):
            self.pose.theta = self.pose.theta + 2*3.14159

        vel_msg = Twist()

        #while (abs(self.pose.theta) - abs(self.__goal_pose.theta)) <= 0.01745:
        while round(self.pose.theta,2) != round(self.__goal_pose.theta,2):
            err = round(self.__goal_pose.theta, 2) - round(self.pose.theta, 2)

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.5 * err

            self.vel_publisher.publish(vel_msg)
            #self.rate.sleep()

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
    pp = [(1,1,90), (1,10,0), (10,10,-90), (10,1,-180), (1,1,0)]
    tt = Turtle()
    #tt.move2goal()
    #tt.rotate_itself()
    tt.rotate_ctrl(-135)
    tt.linear_ctrl([1,1,90])
    #tt.rotate_ctrl(90)
    tt.linear_ctrl([1,10,0])
    #tt.rotate_ctrl(0)
    tt.linear_ctrl([10,10,-90])
    #tt.rotate_ctrl(-90)
    #for point in pp:
        #print(point[2])
    
    #for point in pp:
        #tt.conti_path(point)
        #tt.rotate_ctrl(-135)
        #tt.linear_ctrl(point)
        #tt.rotate_ctrl(point[2])
    
