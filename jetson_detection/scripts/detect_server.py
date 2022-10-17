#!/usr/bin/python3
import cv2
import torch
import rospy

def predict_fn(image, resolution=256):
    result = model(image, size=resolution)
    if len(result.xyxy[0].cpu().numpy().tolist()) > 0:
        return result.xyxy[0].cpu().numpy().tolist()[0][-1]
    return -1
        
def most_frequent(prediction_list):
    counter = 0
    num = prediction_list[0]
    for i in prediction_list:
        curr_frequency = prediction_list.count(i)
        if(curr_frequency> counter):
            counter = curr_frequency
            num = i
    return num
    
if __name__ == "__main__":
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
        dispW = 640
        dispH = 480
        flip  = 2
        # Camera Capture Settings
        capW = 1280
        capH = 720
        fps = 60
        # camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width='+str(capW)+', height='+str(capH)+', format=NV12, framerate='+str(fps)+'/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        cam = cv2.VideoCapture(camSet)

        predictions = []
        n = 0
        while not rospy.is_shutdown():
            ret, frame = cam.read()
            print("Processing")
            if ret:
                print("Returned Pic")
                p = predict_fn(frame)
                print("Appending Predictions")
                predictions.append(p)

                if n >= 3:
                    n = 0
                    predict = most_frequent(predictions)
                    predictions.clear()
                #     # if predict ==  0 : with_cap
                #     # if predict ==  1 : without_cap
                #     # if predict == -1 : Unknown
                    rospy.loginfo("Result: %d" %predict)

            n += 1
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()   

