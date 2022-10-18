#!/usr/bin/env python3
import torch
import rospy
import numpy as np
from sensor_msgs.msg import Image

class Process_image:
    _predictions = []
    n_frames = 0
    
    def __init__(self):
        ## Load Object Detection Model
        model_path = rospy.get_param('~model_path')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  
        self.model.conf = 0.8

    def img_cb(self,image_data):
        rospy.loginfo("Image Recieved")
        self.n_frames += 1

        cv_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        
        proc_img = self.predict_fn(cv_image)
        rospy.loginfo(proc_img)
        rospy.loginfo("Done Processing")
        # self._predictions.append(proc_img)

        # if self.n_frames >= 3:
        #     self.most_frequent()

    def predict_fn(self, image, resolution=256):
        rospy.loginfo("Processing Image")
        result = self.model(image, size=resolution)
        output = len(result.xyxy[0].cpu().numpy().tolist())
        if output > 0:
            return result.xyxy[0].cpu().numpy().tolist()[0][-1]
        else:
            return -1
        
            
    def most_frequent(self):
        rospy.loginfo("Averaging Result")
        counter = 0
        num = self._predictions[0]
        for i in self._predictions:
            curr_frequency = self._predictions.count(i)
            if(curr_frequency> counter):
                counter = curr_frequency
                num = i
        ## Clear the list
        self.n_frames = 0
        self._predictions.clear()
        rospy.loginfo(num)
    
if __name__ == "__main__":
    try:
        rospy.init_node('detect_server')
        proc = Process_image()
        rospy.Subscriber('/camera1/raw_image', Image, proc.img_cb)
        rospy.loginfo("Ready to Process any image")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

