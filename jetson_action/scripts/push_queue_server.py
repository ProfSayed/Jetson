#!/usr/bin/env python3
import rospy
import actionlib
from jetson_msgs.msg import CountSensor
from jetson_msgs.msg import PushAction
from jetson_msgs.msg import PushResult
from jetson_msgs.msg import PushFeedback
from jetson_msgs.srv import Actuator

def pusher_client(action):
    pusher_service = rospy.get_param('/pusher_server/topic_name')
    rospy.wait_for_service(pusher_service)
    try:
        push_cylinder = rospy.ServiceProxy(pusher_service, Actuator)
        resp1 = push_cylinder(action)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

class CylinderPushServer:
    _feedback = PushFeedback()
    _result = PushResult()

    def __init__(self):
        pusher_server = rospy.get_param('/push_action_server/server_name')
        self._as = actionlib.ActionServer(pusher_server, PushAction, self.on_goal, self.on_cancel, auto_start=False)
        self._as.start()
        rospy.loginfo("Push Queue Action Server Started")
        
    def pusher_callback(self, data):
        self.pusher_count = data.cylinder_number

    def process_goal(self, goal_handle):
        goal = goal_handle.get_goal()
        cylinder_number = goal.cylinder_number
        rospy.loginfo("Pushing %d"%cylinder_number)

        # Validate Parameters "Change from Pending State"
        goal_handle.set_accepted()
        if cylinder_number < self.pusher_count:
            rospy.loginfo("The Sensor Shows %d"%self.pusher_count)
            goal_handle.set_rejected()
            return
        
        success = False
        preempted = False
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            # if the Cylinder is Pushed "Success"
            if cylinder_number == self.pusher_count:
                pusher_client(True)
                success = True
                break
            self._feedback.cylinder_reached = self.pusher_count
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        rospy.sleep(2)
        pusher_client(False)
        self._result.cylinder_kicked = cylinder_number
        self._result.time_stamp      = rospy.Time.now()

        if preempted:
            rospy.loginfo("Preempted")
            goal_handle.set_canceled(self._result)
        elif success:
            rospy.loginfo("Succeeded Pushing %s"%cylinder_number)
            goal_handle.set_succeeded(self._result)
        else:
            rospy.loginfo("Aborted")
            goal_handle.set_aborted(self._result)
        rospy.loginfo("--------Goal Processing Finished")
     
    def on_goal(self, goal_handle):
        rospy.loginfo("Recieved new goal")
        self.process_goal(goal_handle)
    
    def on_cancel(self, goal_handle):
        rospy.loginfo("Recieved cancel request")

if __name__ == "__main__":
    rospy.init_node("cylinder_pusher_server")
    server = CylinderPushServer()
    ## Pusher_Sensor Subscriber
    pusher_sub = rospy.Subscriber('/pusher_count', CountSensor, server.pusher_callback)

    rospy.spin()