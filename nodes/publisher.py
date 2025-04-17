#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"

IMAGE = Image()

def cam_callback(msg):
  global IMAGE
  IMAGE.height = msg.height
  IMAGE.width = msg.width
  IMAGE.encoding = msg.encoding
  IMAGE.step = msg.step
  IMAGE.data = msg.data

def generate_image(width: int = 320, height: int = 240) -> np.ndarray:
    return np.random.randint(0, 256, (height, width), dtype=np.uint8)

def main() -> None:
  global IMAGE
  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
  # A: потому что тут уже разрешаются имена.
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)
  # subscriber = rospy.Subscriber("pylon_camera_node/image_raw", Image, cam_callback)
  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

  rate = rospy.Rate(pub_frequency)
  bridge = CvBridge()

  
  while not rospy.is_shutdown():
    
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    
    image = generate_image()
        
    msg = bridge.cv2_to_imgmsg(image, encoding="mono8")
    publisher.publish(msg)
        
    rate.sleep()
  

if __name__ == '__main__':
    main()
