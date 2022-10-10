#!/usr/bin/python3
import cv2
import torch
import numpy as np
import rospy
import actionlib
from jetson_msgs.msg import DetectAction
from jetson_msgs.msg import DetectResult
from jetson_msgs.msg import DetectFeedback

class DetectionAction(object):
    # create messages that are used to publish feedback/result
    _feedback = DetectFeedback()
    _result = DetectResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, DetectAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.predictions = []
        ## Load Object Detection Model
        model_path = rospy.get_param('~model_path')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  
        self.model.conf = 0.7

        ## Camera
        # Camera Display Settings
        dispW = 1280
        dispH = 720
        flip  = 2
        # Camera Capture Settings
        capW = rospy.get_param('~camera/capture_width')
        capH = rospy.get_param('~camera/capture_height')
        fps = rospy.get_param('~camera/fps')
        camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width='+str(capW)+', height='+str(capH)+', format=NV12, framerate='+str(fps)+'/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        self.cam = cv2.VideoCapture(camSet)

    def predict_fn(self, image, model, resolution):
        predictions = -1
        result = model(image, size=resolution)
        output = result.xyxy[0].cpu().numpy().tolist()

        if len(output) > 0:
            predictions = output[0][-1]
        return predictions
            
    def most_frequent(self, prediction_list):
        counter = 0
        num = prediction_list[0]
        for i in prediction_list:
            curr_frequency = prediction_list.count(i)
            if(curr_frequency> counter):
                counter = curr_frequency
                num = i
        return num
        
      
    def execute_cb(self, goal):
        ## Detection Frequency
        detection_rate = rospy.get_param('~frequency')
        rate = rospy.Rate(detection_rate)
        success = True
        rospy.loginfo('%s: Executing, Processing cylinder number %d ' % (self._action_name, goal.cylinder_number))

        # start executing the action
        for i in range(0, goal.number_of_frames +1):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            ret, frame = self.cam.read()
            if ret:
                p  = self.predict_fn(frame, self.model, 240)
                self.predictions.append(p)

                ## Feedback
                # self._feedback.frame_processed = i
                # self._as.publish_feedback(self._feedback)

            rate.sleep()
        ## Exit the Loop
        predict = self.most_frequent(self.predictions)
        self.predictions.clear()
        
        if success:
            self._result.cylinder_number = goal.cylinder_number
            self._result.has_cap = predict
            self._result.time_stamp = rospy.Time.now()
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    try:
        rospy.init_node('detect_server')
        name = rospy.get_param('~server_name')
        server = DetectionAction(name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        server.cam.release()        
