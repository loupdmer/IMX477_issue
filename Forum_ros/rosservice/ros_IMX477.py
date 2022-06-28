#!/usr/bin/env python3
#Author : Dorian Ibert
#Last update : June 2022

import rospy
import os
import sys
from pathlib import Path

import cv2
import time
import datetime
import threading

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()

# Initialize architecture
FILE = Path(sys.argv[0]).resolve() # Get the full path of the file
node_name = os.path.splitext(FILE.name)[0]

import rospkg
rospack = rospkg.RosPack()
package = rospkg.get_package_name(FILE) # Get the package name
ROOT = Path(package).resolve() # Get the full path of the package

from vision.srv import CameraResolution, CameraResolutionResponse, CameraOnOff, CameraOnOffResponse

# Topics to subscribe/publish
camera_topic = "/cam_2/color/image_raw"

# Define a camera handler
class Camera_handler:

    def __init__(self):
        # Create the camera handler object
        
        try:
            entries = os.listdir("/dev")
            num = min([int(entry[5:]) for entry in entries if "video" in entry])
            src = "/dev/video"+str(num)
            self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        except cv2.error as error:
            print("[Error]: {}".format(error))
            
        if not self.cap.isOpened():
            print("Cannot open camera handler")
            exit() 
        
        # Init parameters
        (self.ret, self.img) = self.cap.read()
        self.idx = 1 # The number of frame grabbed
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)

        rospy.set_param('imx477', {'width': self.width, 'height': self.height, 'fps': self.fps})
        
        self.cam_pub = rospy.Publisher(camera_topic, Image, queue_size=1) # Publisher
        print("Camera handler capturing %s at (%d,%d) %dFPS" %(src, int(self.width),int(self.height),int(self.fps)))
        self.ResolutionService = rospy.Service('setResolution', CameraResolution, self.setResolution)
        self.CaptureService = rospy.Service('Capture', CameraOnOff, self.Capture)
        self.mutex = threading.Semaphore()

    def read(self):
        self.mutex.acquire()
        (self.ret, self.img) = self.cap.read()
        if self.ret:
            self.img = cv2.rotate(self.img, cv2.ROTATE_90_CLOCKWISE)
            im = bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
            
            # Select one frame according to the index and publish it
            im.header.stamp = rospy.get_rostime() # Set a timestamp
            im.header.frame_id = "frame_IMX477"
            im.header.seq = self.idx
            self.cam_pub.publish(im) # Publish the frame
            #rospy.loginfo("frame published")
            self.idx+=1
        self.mutex.release()

    def setResolution(self, req):
        resolution = round(req.resolution,1)
        if resolution == 0.3:
            width, height = 640, 480
        elif resolution == 2:
            width, height = 1920, 1080
        elif resolution == 5:
            width, height = 2592, 1944
        elif resolution == 8:
            width, height = 3840, 2160
        elif resolution == 12:
            width, height = 4032, 3040
        else:
            print("Required resolution %.1fMpx unavailable" %resolution)
            return CameraResolutionResponse(False)

        try :

            self.mutex.acquire()
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # update attributes
            self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)
            rospy.set_param('imx477', {'width': self.width, 'height': self.height, 'fps': self.fps})

            print("Parameters updated to (%d,%d) %dFPS" %(int(self.width),int(self.height),int(self.fps)))
            self.mutex.release()
            return CameraResolutionResponse(True)
        except:
            self.mutex.release()
            return CameraResolutionResponse(False)

    def Capture(self, req):
        self.mutex.acquire()
        (self.ret, self.img) = self.cap.read()
        if self.ret:
            self.img = cv2.rotate(self.img, cv2.ROTATE_90_CLOCKWISE)
            filepath = str(req.path)+'/'+'led_'+str(datetime.datetime.now())+'.jpg'
            cv2.imwrite(filepath, self.img)
            self.mutex.release()
            rospy.loginfo("Image saved in %s" %filepath)
            return CameraOnOffResponse(True)
        else:
            rospy.loginfo("Failed saving frame")
            self.mutex.release()
            return CameraOnOffResponse(False)

    def release(self):
        self.cap.release()
        print("VideoCapture released")



def IMX477_usb():
    # Create the camera controller
    imx = Camera_handler()
    #r = rospy.Rate(50)
    while not rospy.is_shutdown():
        imx.read()
        #r.sleep()
    imx.release()
    
    
if __name__ == '__main__':

    rospy.init_node(node_name, anonymous = False)
    print("Node %s initialized" %rospy.get_name())
    IMX477_usb()
    
