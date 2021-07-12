#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2

if __name__ == '__main__':
    rospy.init_node('camera_publisher',anonymous=True)
    pub = rospy.Publisher('camera_image', Image, queue_size = 10)
    cap =cv2.VideoCapture(0)
    print('FPS: ',cap.get(cv2.CAP_PROP_FPS))
    print('W: ',cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print('H: ',cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret,frame=cap.read()
        frame=cv2.flip(frame,0)		
        cv2.imshow('1',frame)
        #pub.publish(frame)
        rate.sleep()


