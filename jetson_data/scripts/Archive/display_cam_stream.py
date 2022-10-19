#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera1/raw_image",Image,self.callback)

  def callback(self,data):
    # try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #   print(e)
    
    ## anoter Way
    cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()