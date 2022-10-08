#!/usr/bin/python3
import cv2
import torch
import rospy

def predict(image, model, resolution):
    predictions = -1
    result = model(image, size=resolution)
    output = result.xyxy[0].cpu().numpy().tolist()

    if len(output) > 0:
        predictions = output[0][-1]
    return predictions
        
def most_frequent(prediction_list):
    counter = 0
    num = prediction_list[0]
    for i in prediction_list:
        curr_frequency = prediction_list.count(i)
        if(curr_frequency> counter):
            counter = curr_frequency
            num = i
    return num

try:
    rospy.init_node('detect_server')
    ## Detection Frequency
    detection_rate = rospy.get_param('~frequency')
    rate = rospy.Rate(detection_rate)

    ## Load Object Detection Model
    model_path = rospy.get_param('~model_path')
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)  
    model.conf = 0.4

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
    cam = cv2.VideoCapture(camSet)

    predictions = []
    n = 0
    while not rospy.is_shutdown():
        ret, frame = cam.read()
        print("Processing")
        # if ret:
        #     p = predict(frame, model, 640)
        #     predictions.append(p)
        #     print("Appending Predictions")


        # if n == 3:
        #     n = 0
        #     predict = most_frequent(predictions)
        #     predictions.clear()
        #     # if predict ==  0 : with_cap
        #     # if predict ==  1 : without_cap
        #     # if predict == -1 : Unknown
        #     rospy.loginfo("Result: %d" %predict)

        n += 1
        rate.sleep()

except rospy.ROSInterruptException:
    pass
finally:
    cam.release()   