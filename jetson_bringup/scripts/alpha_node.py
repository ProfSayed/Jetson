#!/usr/bin/env python3
import rospy
import actionlib
from jetson_msgs.msg import DetectAction
from jetson_msgs.msg import DetectGoal
from jetson_msgs.msg import PushAction
from jetson_msgs.msg import PushGoal
from jetson_msgs.msg import CountSensor
from jetson_msgs.srv import Actuator

def stopper_client(action):
    stopper_service = rospy.get_param('/stopper_server/topic_name')
    rospy.wait_for_service(stopper_service)
    try:
        stop_cylinder = rospy.ServiceProxy(stopper_service, Actuator)
        resp1 = stop_cylinder(action)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

class Client:
    def __init__(self):
        detect_server = rospy.get_param('/detect_server/server_name')
        self._ac_detect = actionlib.SimpleActionClient(detect_server, DetectAction)
        self._ac_detect.wait_for_server()
        rospy.loginfo("Detection Action Client is ready")

        pusher_server = rospy.get_param('/push_action_server/server_name')
        self._ac_push = actionlib.SimpleActionClient(pusher_server, PushAction)
        self._ac_push.wait_for_server()
        rospy.loginfo("Pusher Action Client is ready")
      
    def countsensor_callback(self,data):
        # Called whenever a cylinder is detected by the sensor 
        ## Stop Cylinder - True to Stop, False to Pass
        ### 1. Stop the Cylinder
        stopper_client(True)
        rospy.loginfo("The Cylinder has been stopped")

        ### 2. Identify the Cylinder
        goal = DetectGoal(cylinder_number=data.cylinder_number, time_stamp=rospy.Time.now())
        self._ac_detect.send_goal(goal, done_cb=self.detect_done_callback, feedback_cb=self.detect_feedback_callback)
        rospy.loginfo("Detection Goal has been sent")
    
    def detect_done_callback(self, status, result):
        rospy.loginfo("Status is: %s" %status)
        rospy.loginfo("Result is: %s" %result)

        ### 3. Release the Cylinders
        stopper_client(False)

        ### 4. Push if it's unknown object or it has no cap
        if result.has_cap != 0:
            goal = PushGoal(cylinder_number=result.cylinder_number)
            self._ac_push.send_goal(goal, done_cb=self.push_done_callback, feedback_cb=self.push_feedback_callback)
            rospy.loginfo("Pusher Goal has been sent")

    def detect_feedback_callback(self, feedback):
        rospy.loginfo(feedback)

    def push_done_callback(self, status, result):
        rospy.loginfo("Status is: %s" %status)
        rospy.loginfo("Result is: %s" %result)

    def push_feedback_callback(self, feedback):
        rospy.loginfo(feedback)

def main():
    rospy.init_node('alpha_node')
    client = Client()
    stopper_topic = rospy.get_param('/stopper_sensor/topic_name')
    rospy.Subscriber(stopper_topic, CountSensor, client.countsensor_callback)
    rospy.spin()

if __name__ == '__main__':
    main()