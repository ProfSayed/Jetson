#!/usr/bin/python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
    cv_image = cv2.imread('~/catkin_ws/src/Jetson/jetson_data/samples/cylinder.png')   
    try:
        rospy.init_node('cam_streamer')
        bridge = CvBridge()
        img_pub = rospy.Publisher('/camera1/raw_image', Image,queue_size=3)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rospy.loginfo_once("Started Capturing")
            try:
                img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                img_pub.publish(img_msg)
            except CvBridgeError as e:
                rospy.logerr(e)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
