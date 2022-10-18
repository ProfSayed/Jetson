#!/usr/bin/env python
# import torch
import rospy
import cv2

_predictions = []
n_frames = 0

def predict_fn(image, resolution=256):
    global n_frames, _predictions, model
    rospy.loginfo("Processing Image")
    result = model(image, size=resolution)
    output = len(result.xyxy[0].cpu().numpy().tolist())
    if output > 0:
        return result.xyxy[0].cpu().numpy().tolist()[0][-1]
    else:
        return -1
    
        
def most_frequent():
    global n_frames, _predictions
    rospy.loginfo("Averaging Result")
    counter = 0
    num = _predictions[0]
    for i in _predictions:
        curr_frequency = _predictions.count(i)
        if(curr_frequency> counter):
            counter = curr_frequency
            num = i
    ## Clear the list
    n_frames = 0
    _predictions.clear()
    rospy.loginfo(num)


if __name__ == "__main__":
    try:
        rospy.init_node('detect_server')
        rate = rospy.Rate(20)

        ## Camera
        # Camera Display Settings
        dispW = 256
        dispH = 256
        flip  = 2 ## 0 or 2
        # Camera Capture Settings
        capW = 3264
        capH = 2464
        fps = 21
        camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width='+str(capW)+', height='+str(capH)+', format=NV12, framerate='+str(fps)+'/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        cam = cv2.VideoCapture(camSet)

        # ## Load Object Detection Model
        # model_path = rospy.get_param('~model_path')
        # model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  
        # model.conf = 0.8


        while not rospy.is_shutdown():
            ret, img = cam.read()
            rospy.loginfo_once("Started Loop")
            if ret:
                n_frames += 1
                # rospy.loginfo("Image Recieved")
                # proc_img = predict_fn(img)
                # rospy.loginfo(proc_img)
                # rospy.loginfo("Done Processing")

                # self._predictions.append(proc_img)

                # if self.n_frames >= 3:
                #     self.most_frequent()
            
            rate.sleep()      
        
    except rospy.ROSInterruptException:
        pass
    cam.release()   

        

