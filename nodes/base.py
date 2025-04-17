#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


img = Image()

def cam_callback(msg):
    global img
    img.height = msg.height
    img.width = msg.width
    img.encoding = msg.encoding
    img.step = msg.step
    img.data = msg.data

def main():
    global img
    rospy.init_node("base")

    publisher = rospy.Publisher("/usb_cam/image_color", Image, queue_size=10)    
    subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, cam_callback)
    while not rospy.is_shutdown():
        publisher.publish(img) 

  

if __name__ == '__main__':
    main()
