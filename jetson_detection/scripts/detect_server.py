#!/usr/bin/env python3
import cv2
import torch
import rospy
import numpy as np
from jetson_msgs.srv import Detect ,DetectResponse

class Process_image:
    def __init__(self):
        ## Load Object Detection Model
        model_path = rospy.get_param('~model_path')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  
        self.model.conf = rospy.get_param('~model_config')

    def prcoess_img(self,req):
        rospy.loginfo("Image Recieved")
        image_data = req.raw_image
        cv_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        
        ## Save img
        n = 0
        filename = n + '.jpg'
        cv2.imwrite(filename, cv_image)
        n += 1

        result = self.model(cv_image, size=256)
        output = result.xyxy[0].cpu().numpy().tolist()
        if len(output) > 0:
            rospy.loginfo("Result: %d" %int(output[0][-1]))
            return DetectResponse(int(output[0][-1]))
        else:
            return DetectResponse(-1)
    
if __name__ == "__main__":
    try:
        rospy.init_node('detect_server')
        topic_name = rospy.get_param('~topic_name')
        proc = Process_image()
        ## Start Server
        rospy.Service(topic_name, Detect, proc.prcoess_img)
        rospy.loginfo("Ready to Process image")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

