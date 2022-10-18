#!/usr/bin/env python3
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
    rospy.init_node("Image_Publisher",anonymous=True)
    rospack = rospkg.RosPack()
    image_path = rospack.get_path('jetson_data') + '/samples/cyl.png'
    cv_image = cv2.imread(image_path)   
    # cv_image = cv2.resize(cv_image,(256,256))
    try:
        bridge = CvBridge()
        img_pub = rospy.Publisher('/camera2/raw_image', Image,queue_size=3)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rospy.loginfo_once("Publishing Image")
            try:
                img_msg = bridge.cv2_to_imgmsg(cv_image,encoding="passthrough")
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
