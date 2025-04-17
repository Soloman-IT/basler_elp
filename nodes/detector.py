#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imutils
from skimage.metrics import structural_similarity as compare_ssim
from hw_basler.msg import Custom


img_old = Image()
img_curr = Image()

count = 0

img_old_ = 0
img_curr_ = 0

bridge = CvBridge()

def cam_callback(msg):
    global img_old, img_curr, bridge, img_old_, img_curr_, count, bridge
    img_curr = msg

    if count == 0:
        img_old_ = cv2.cvtColor(bridge.imgmsg_to_cv2(img_curr, "bgr8"), cv2.COLOR_BGR2GRAY)
        img_curr_ = cv2.cvtColor(bridge.imgmsg_to_cv2(img_curr, "bgr8"), cv2.COLOR_BGR2GRAY)
        count += 1
    else:
        img_old_ = cv2.cvtColor(bridge.imgmsg_to_cv2(img_old, "bgr8"), cv2.COLOR_BGR2GRAY)
        img_curr_ = cv2.cvtColor(bridge.imgmsg_to_cv2(img_curr, "bgr8"), cv2.COLOR_BGR2GRAY)

    img_old = img_curr

def compute_diff():
    global img_old_, img_curr_
    diff = np.sum(cv2.absdiff(img_old_, img_curr_))
    return diff

def main():
    global img, img_curr_
    rospy.init_node("base")

    publisher_custom = rospy.Publisher("/usb_cam/motion_detection", Custom, queue_size=10)    
    
    subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, cam_callback)
    while not rospy.is_shutdown():
        msg = Custom()
        
        diff = compute_diff()        
        if diff < 450000:
            msg.detected = False
        else:
            msg.detected = True
        msg.diff = diff
        publisher_custom.publish(msg)    
        print("!")

if __name__ == '__main__':
    main()
