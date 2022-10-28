#!/usr/bin/env python
import cv2
import rospy
import rospkg
from jetson_msgs.srv import Detect
from cv_bridge import CvBridge, CvBridgeError
    
if __name__ == "__main__":
    try:
        rospy.init_node('detect_client')
        ## Prepare image
        rospack = rospkg.RosPack()
        image_path = rospack.get_path('jetson_detection') + '/samples/cyl.png'
        cv_image = cv2.imread(image_path)   
        cv_image = cv2.resize(cv_image,(256,256))
        bridge = CvBridge()
        try:
            img_msg = bridge.cv2_to_imgmsg(cv_image,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        ## Send it to server
        topic_name = rospy.get_param('detect_server/topic_name')
        rospy.wait_for_service(topic_name)
        request = rospy.ServiceProxy(topic_name,Detect)
        respond = request(img_msg)
        rospy.loginfo(respond)

    except rospy.ROSInterruptException:
        pass

