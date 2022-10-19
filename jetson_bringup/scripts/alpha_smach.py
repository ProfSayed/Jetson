#!/usr/bin/env python
import rospy
import smach
import smach_ros
from sensor_msgs.msg import Image
from jetson_msgs.msg import CountSensor
from jetson_msgs.srv import Actuator, ActuatorRequest
from jetson_msgs.srv import Detect

class Detect_avg(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','more_frames'], input_keys=['has_cap'],output_keys=['avg_cap'])

    def execute(self, ud):
        rospy.loginfo("Cylinder has %s"% ud.has_cap)
        rospy.loginfo(ud.counter)
        ud.counter += 1
        if ud.counter < 10:
            return 'more_frames'
        else:
            ud.counter = 0
            return 'succeeded'

def stopper_sensor_cb(ud, msg):
    rospy.loginfo('Cylinder Detected by Sensor')
    ud.cylinder_number = msg.cylinder_number
    return True 
    
def capture_img_cb(ud, msg):
    rospy.loginfo('Image Recieved')
    ud.raw_image = msg
    return True 


### ==================== main ==================== ###
def main():
    rospy.init_node('alpha_smach')

    ## Param
    raw_img_topic_name = rospy.get_param('/raw_image_topic_name')
    ss_topic_name = rospy.get_param('/stopper_sensor/topic_name')
    ps_topic_name = rospy.get_param('/pusher_sensor/topic_name')
    pusher_service_name = rospy.get_param('/pusher_server/topic_name')
    stopper_service_name = rospy.get_param('/stopper_server/topic_name')

    ## RESET Group
    sm_reset = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    with sm_reset:
        smach.StateMachine.add('RESET_PUSHER' , smach_ros.ServiceState(pusher_service_name, Actuator , request=ActuatorRequest(False)), transitions={'succeeded':'RESET_STOPPER'})
        smach.StateMachine.add('RESET_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)))
    
    ## Object Detection Group
    sm_detect = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    detect_service_name = rospy.get_param('/detect_server/topic_name')
    with sm_detect:
        sm_detect.userdata.counter = 0
        smach.StateMachine.add('CAP_FRAME', smach_ros.MonitorState(raw_img_topic_name, Image, capture_img_cb, 1, output_keys=['raw_image']), transitions={'invalid':'CAP_FRAME', 'valid':'DETECT_FRAME'})
        smach.StateMachine.add('DETECT_FRAME' , smach_ros.ServiceState(detect_service_name, Detect , request_slots=['raw_image'], response_slots=['has_cap']), transitions={'succeeded':'DETECT_AVG'})
        smach.StateMachine.add('DETECT_AVG',Detect_avg(), transitions={'more_frames':'CAP_FRAME','succeeded':'succeeded'})

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    with sm_top:
        smach.StateMachine.add('RESET',sm_reset, transitions={'succeeded':'MONITOR_SS'})
        smach.StateMachine.add('MONITOR_SS', smach_ros.MonitorState(ss_topic_name, CountSensor, stopper_sensor_cb, 1, output_keys=['cylinder_number']), transitions={'invalid':'MONITOR_SS', 'valid':'STOPPER_ACTION'})
        smach.StateMachine.add('STOPPER_ACTION', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(True)), transitions={'succeeded':'DETECT'})
        smach.StateMachine.add('DETECT',sm_detect, transitions={'succeeded':'RELEASE_STOPPER'})
        smach.StateMachine.add('RELEASE_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)))
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(rospy.get_name(), sm_top, '/JETSON_SYS')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()