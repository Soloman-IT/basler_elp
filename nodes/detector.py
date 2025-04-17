#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFilter:
    def __init__(self):
        rospy.init_node('color_filter_node')
        
        # Получение параметра цвета
        self.color = rospy.get_param('~color', 'красный').lower()
        
        # Инициализация CvBridge
        self.bridge = CvBridge()
        
        # Определение диапазонов HSV для цветов
        self.set_hsv_ranges()
        
        # Публикатор для маски
        self.mask_pub = rospy.Publisher('/color_mask', Image, queue_size=10)
        
        # Подписчик на изображение с камеры
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        rospy.loginfo(f"Фильтр цвета запущен для цвета: {self.color}")

    def set_hsv_ranges(self):
        if self.color == 'красный':
            self.lower1 = np.array([0, 100, 100])
            self.upper1 = np.array([10, 255, 255])
            self.lower2 = np.array([160, 100, 100])
            self.upper2 = np.array([180, 255, 255])
        elif self.color == 'зеленый':
            self.lower = np.array([40, 50, 50])
            self.upper = np.array([80, 255, 255])
        elif self.color == 'синий':
            self.lower = np.array([100, 50, 50])
            self.upper = np.array([140, 255, 255])
        else:
            rospy.logerr(f"Неизвестный цвет: {self.color}. Допустимые цвета: красный, зеленый, синий.")
            rospy.signal_shutdown("Некорректный параметр цвета")

    def image_callback(self, msg):
        try:
            # Конвертация ROS Image в OpenCV Image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Конвертация в HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Создание маски
            if self.color == 'красный':
                mask1 = cv2.inRange(hsv_image, self.lower1, self.upper1)
                mask2 = cv2.inRange(hsv_image, self.lower2, self.upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_image, self.lower, self.upper)
            
            # Морфологические операции для улучшения маски
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1)
            
            # Публикация маски
            mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
            self.mask_pub.publish(mask_msg)
            
        except Exception as e:
            rospy.logerr(f"Ошибка обработки изображения: {e}")

if __name__ == '__main__':
    try:
        node = ColorFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass