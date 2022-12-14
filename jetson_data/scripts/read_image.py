#!/usr/bin/env python
import cv2
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node("Image_Publisher")
    rospack = rospkg.RosPack()
    image_path = rospack.get_path('jetson_detection') + '/samples/cyl.png'
    cv_image = cv2.imread(image_path)   
    cv_image = cv2.resize(cv_image,(256,256))
    try:
        bridge = CvBridge()
        topic_name = rospy.get_param('/raw_image_topic_name')
        img_pub = rospy.Publisher(topic_name, Image,queue_size=1)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rospy.loginfo_once("Publishing Image")
            try:
                img_msg = bridge.cv2_to_imgmsg(cv_image,"bgr8")
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
