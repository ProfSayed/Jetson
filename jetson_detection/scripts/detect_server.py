#!/usr/bin/env python3
import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Process_image:
    _predictions = []
    n_frames = 0
    
    def __init__(self):
        self.bridge = CvBridge()

    def img_cb(self,data):
        rospy.loginfo("Image Recieved")
        self.n_frames += 1
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        self.predict_fn(img)

        if self.n_frames >= 3:
            self.most_frequent()

    def predict_fn(self, image, resolution=256):
        rospy.loginfo("Processing Image")
        result = model(image, size=resolution)
        p = -1 
        if len(p) > 0:
            p = result.xyxy[0].cpu().numpy().tolist()[0][-1]
        self._predictions.append(p)
            
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
        ## Load Object Detection Model
        model_path = rospy.get_param('~model_path')
        # model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  
        model.conf = 0.8

        proc = Process_image()
        rospy.Subscriber('/camera1/raw_image', Image, proc.img_cb)

        rospy.loginfo("Ready to Process any image")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

