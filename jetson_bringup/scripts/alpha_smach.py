#!/usr/bin/env python
import rospy
import smach
import smach_ros
from sensor_msgs.msg import Image
from jetson_msgs.msg import CountSensor
from jetson_msgs.srv import Actuator, ActuatorRequest
from jetson_msgs.srv import Detect, DetectRequest

def stopper_sensor_cb(ud, msg):
    rospy.loginfo('Cylinder Detected by Sensor')
    return True 
    
def capture_img_cb(ud, msg):
    rospy.loginfo('Image Recieved')
    ud.raw_img = msg
    return True 

# class Example(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success'])
#         self.counter = 0

#     def execute(self, userdata):
#         rospy.loginfo('Object Detection Initalized')
#         rospy.sleep(2)
#         return 'success'



### ==================== main ==================== ###
def main():
    rospy.init_node('alpha_smach')

    ## RESET Group
    sm_reset = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    pusher_service_name = rospy.get_param('/pusher_server/topic_name')
    stopper_service_name = rospy.get_param('/stopper_server/topic_name')
    with sm_reset:
        smach.StateMachine.add('RESET_PUSHER' , smach_ros.ServiceState(pusher_service_name, Actuator , request=ActuatorRequest(False)), transitions={'succeeded':'RESET_STOPPER'})
        smach.StateMachine.add('RESET_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)))
    
    ## Object Detection Group
    sm_detect = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    detect_service_name = rospy.get_param('/detect_server/topic_name')
    with sm_detect:
        smach.StateMachine.add('CAP_FRAME', smach_ros.MonitorState("/sm_reset", Image, capture_img_cb, 1, output_keys=['raw_img']), transitions={'invalid':'CAP_FRAME', 'valid':'DETECT_FRAME'})
        smach.StateMachine.add('DETECT_FRAME' , smach_ros.ServiceState(detect_service_name, Detect , request=DetectRequest(sm_detect.userdata.raw_img)))

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    with sm_top:
        smach.StateMachine.add('RESET',sm_reset, transitions={'succeeded':'MONITOR_SS'})
        smach.StateMachine.add('MONITOR_SS', smach_ros.MonitorState("/sm_reset", CountSensor, stopper_sensor_cb, 1), transitions={'invalid':'MONITOR_SS', 'valid':'STOPPER_ACTION'})
        smach.StateMachine.add('STOPPER_ACTION', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(True)), transitions={'succeeded':'RELEASE_STOPPER'})
        smach.StateMachine.add('RELEASE_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)))
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(rospy.get_name(), sm_top, '/JETSON_SYS')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()