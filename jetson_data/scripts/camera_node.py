#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
    try:
        rospy.init_node('cam_stream')
        bridge = CvBridge()
        topic_name = rospy.get_param('/raw_image_topic_name')
        img_pub = rospy.Publisher(topic_name, Image,queue_size=1)
        rate = rospy.Rate(28)

        ## Camera
        # Camera Display Settings
        dispW = 256
        dispH = 256
        flip  = 2 ## 0 or 2
        # Camera Capture Settings
        capW = 3264
        capH = 1848
        fps = 28
        camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width='+str(capW)+', height='+str(capH)+', format=NV12, framerate='+str(fps)+'/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        cam = cv2.VideoCapture(camSet)

        while not rospy.is_shutdown():
            ret, cv_image = cam.read()
            rospy.loginfo_once("Started Capturing")
            if ret:
                try:
                    img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                    img_pub.publish(img_msg)
                except CvBridgeError as e:
                    rospy.logerr(e)
            else:
                rospy.logerr("Not recieving any frames")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()   

if __name__ == "__main__":
    main()