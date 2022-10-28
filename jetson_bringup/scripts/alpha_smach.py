#!/usr/bin/env python
import rospy
import smach
import smach_ros
from sensor_msgs.msg import Image
from jetson_msgs.msg import CountSensor
from jetson_msgs.srv import Actuator, ActuatorRequest
from jetson_msgs.srv import Detect

class Detect_max(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','more_frames'], input_keys=['n_frames','counter','has_cap','result_list'],output_keys=['counter','has_cap_result','result_list'])

    def execute(self, ud):
        if ud.counter < ud.n_frames:
            ud.counter += 1
            ud.result_list.append(ud.has_cap)
            return 'more_frames'
        else:    
            cnt = 0
            n = ud.result_list[0]
            for i in ud.result_list:
                curr_freq = ud.result_list.count(i)
                if curr_freq > cnt:
                    cnt = curr_freq
                    n = i
            ## Result
            ud.counter = 0
            ud.has_cap_result = n
            return 'succeeded'

class If_cap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['has_cap','no_cap'],input_keys=['has_cap_result'])

    def execute(self, ud):
        if ud.has_cap_result == 0:
            return "has_cap"
        else:
            return "no_cap"

def stopper_sensor_cb(ud, msg):
    rospy.loginfo('Cylinder Detected by Sensor')
    if msg.cylinder_number:
        ud.cylinder_number = msg.cylinder_number
        return False
    else:
        return True
    
def pusher_sensor_cb(ud, msg):
    rospy.loginfo('Cylinder %s Detected by Sensor' % msg.cylinder_number)
    if msg.cylinder_number:
        return False 
    return True
    
def capture_img_cb(ud, msg):
    rospy.loginfo('Image Recieved')
    ud.raw_image = msg
    return True 

@smach.cb_interface(outcomes=['succeeded'])
def timer_cb(ud,t):
    rospy.sleep(t)
    return 'succeeded'

### ==================== main ==================== ###
def main():
    rospy.init_node('alpha_smach')

    ## Params
    # Topics
    raw_img_topic_name = rospy.get_param('/raw_image_topic_name')
    ss_topic_name = rospy.get_param('/stopper_sensor/topic_name')
    ps_topic_name = rospy.get_param('/pusher_sensor/topic_name')
    # Services
    pusher_service_name = rospy.get_param('/pusher_server/topic_name')
    stopper_service_name = rospy.get_param('/stopper_server/topic_name')
    # Timers
    timer_1 = rospy.get_param('/timers/timer_1')
    timer_2 = rospy.get_param('/timers/timer_2')
    timer_3 = rospy.get_param('/timers/timer_3')
    timer_4 = rospy.get_param('/timers/timer_4')
    timer_5 = rospy.get_param('/timers/timer_5')
    timer_6 = rospy.get_param('/timers/timer_6')
    # Detection
    n_frames = rospy.get_param('/detect_server/number_of_frames_proc')

    ## RESET Group
    sm_reset = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    with sm_reset:
        smach.StateMachine.add('RESET_PUSHER' , smach_ros.ServiceState(pusher_service_name, Actuator , request=ActuatorRequest(False)), transitions={'succeeded':'RESET_STOPPER'})
        smach.StateMachine.add('RESET_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)))
    
    ## Object Detection Group
    sm_detect = smach.StateMachine(outcomes=['succeeded','preempted','aborted'], output_keys=['has_cap_result'])
    detect_service_name = rospy.get_param('/detect_server/topic_name')
    with sm_detect:
        sm_detect.userdata.counter = 0
        sm_detect.userdata.has_cap_result = -1
        sm_detect.userdata.n_frames = n_frames
        sm_detect.userdata.result_list = []
        smach.StateMachine.add('CAP_FRAME', smach_ros.MonitorState(raw_img_topic_name, Image, capture_img_cb, 1, output_keys=['raw_image']), transitions={'invalid':'CAP_FRAME', 'valid':'DETECT_FRAME'})
        smach.StateMachine.add('DETECT_FRAME' , smach_ros.ServiceState(detect_service_name, Detect , request_slots=['raw_image'], response_slots=['has_cap']), transitions={'succeeded':'DETECT_RESULT'})
        smach.StateMachine.add('DETECT_RESULT', Detect_max(), transitions={'more_frames':'CAP_FRAME','succeeded':'succeeded'})

    ## PUSHER Group
    sm_push = smach.StateMachine(outcomes=['succeeded','preempted','aborted'], input_keys=['cylinder_number'])
    with sm_push:
        smach.StateMachine.add('RELEASE_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)), transitions={'succeeded':'TIMER4'})
        smach.StateMachine.add('TIMER4',smach.CBState(timer_cb,[timer_4]), transitions={'succeeded':'STOPPER_ACTION'})
        smach.StateMachine.add('STOPPER_ACTION', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(True)), transitions={'succeeded':'MONITOR_PS'})
        smach.StateMachine.add('MONITOR_PS', smach_ros.MonitorState(ps_topic_name, CountSensor, pusher_sensor_cb, 1), transitions={'invalid':'TIMER5', 'valid':'MONITOR_PS'})
        smach.StateMachine.add('TIMER5',smach.CBState(timer_cb,[timer_5]), transitions={'succeeded':'PUSHER_ACTION'})
        smach.StateMachine.add('PUSHER_ACTION', smach_ros.ServiceState(pusher_service_name, Actuator, request=ActuatorRequest(True)), transitions={'succeeded':'TIMER6'})
        smach.StateMachine.add('TIMER6',smach.CBState(timer_cb,[timer_6]), transitions={'succeeded':'RETRACT_PUSHER'})
        smach.StateMachine.add('RETRACT_PUSHER', smach_ros.ServiceState(pusher_service_name, Actuator, request=ActuatorRequest(True)))
   
    ## STOPPER Group
    sm_stop = smach.StateMachine(outcomes=['succeeded','preempted','aborted'], input_keys=['cylinder_number'],output_keys=['cylinder_number'])
    with sm_stop:
        
        smach.StateMachine.add('MONITOR_SS', smach_ros.MonitorState(ss_topic_name, CountSensor, stopper_sensor_cb, 1, input_keys=['cylinder_number'], output_keys=['cylinder_number']), transitions={'invalid':'TIMER1', 'valid':'MONITOR_SS'})
        smach.StateMachine.add('TIMER1',smach.CBState(timer_cb,[timer_1]), transitions={'succeeded':'STOPPER_ACTION'})
        smach.StateMachine.add('STOPPER_ACTION', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(True)), transitions={'succeeded':'succeeded'})
   
    ## Parent State
    sm_top = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    with sm_top:
        sm_top.userdata.cylinder_number = 0
        # smach.StateMachine.add('TIMER',smach.CBState(timer_cb,[t_sensor_pusher]), transitions={'succeeded':'PUSHER_ACTION'})
        smach.StateMachine.add('RESET',sm_reset, transitions={'succeeded':'STOPPER_GROUP'})
        smach.StateMachine.add('STOPPER_GROUP',sm_stop, transitions={'succeeded':'TIMER2'})
        smach.StateMachine.add('TIMER2',smach.CBState(timer_cb,[timer_2]), transitions={'succeeded':'DETECT'})
        smach.StateMachine.add('DETECT',sm_detect, transitions={'succeeded':'TIMER3'})
        smach.StateMachine.add('TIMER3',smach.CBState(timer_cb,[timer_3]), transitions={'succeeded':'IF_CAP'})
        smach.StateMachine.add('IF_CAP',If_cap(), transitions={'has_cap':'RELEASE_STOPPER','no_cap':"PUSHER_GROUP"})
        smach.StateMachine.add('RELEASE_STOPPER', smach_ros.ServiceState(stopper_service_name, Actuator, request=ActuatorRequest(False)), transitions={'succeeded':'STOPPER_GROUP'})
        smach.StateMachine.add('PUSHER_GROUP',sm_push, transitions={'succeeded':'STOPPER_GROUP'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(rospy.get_name(), sm_top, '/JETSON_SYS')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()