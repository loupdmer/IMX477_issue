#!/usr/bin/env python3
#Author : Dorian Ibert
#Last update : June 2022


import rospy
import cv2
import os
import sys
from pathlib import Path

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Topics to subscribe/publish
camera_topic = "/cam_2/color/image_raw"

bridge = CvBridge()

'''
def callback(frame):
    global img
    rospy.loginfo("callback in")
    #frame = bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")
    #frame = cv2.resize(frame, RESOLUTION)
    #img = frame.copy()
    rospy.loginfo("callback out")
'''


def main():
    #global img
    rospy.init_node("test", anonymous = False)
    rospy.loginfo("Initializing node %s" %rospy.get_name())
    r = rospy.Rate(4)
    # Create the topic to send the detected frame for supervision
    #cam_sub = rospy.Subscriber(camera_topic, Image, callback)

    # Initialize the mission
    frame = rospy.wait_for_message(camera_topic, Image)
    img = bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")

    rospy.loginfo("Node %s initialized" %rospy.get_name())
    while not rospy.is_shutdown():
        rospy.loginfo("1st if in")
        if not img is None:
            rospy.loginfo("2nd if in")
            rospy.loginfo("2nd if out")
        rospy.loginfo("1st if out")
        r.sleep()
    print("Exit")

if __name__ == '__main__':
    main()