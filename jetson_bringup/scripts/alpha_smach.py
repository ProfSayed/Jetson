#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from jetson_msgs.msg import DetectAction
from jetson_msgs.msg import DetectGoal
from jetson_msgs.msg import PushAction
from jetson_msgs.msg import PushGoal
from jetson_msgs.msg import CountSensor
from jetson_msgs.srv import Actuator, ActuatorRequest

# Container#1: Sensor,Camera,Stopper Group
def stopper_sensor_cb(ud, msg):
    rospy.loginfo('Cylinder Detected by Sensor')
    return True 

class Detect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Object Detection Initalized')
        rospy.sleep(2)
        return 'success'

class Release(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Cylinder Released')
        rospy.sleep(2)
        return 'success'

# Container#2: Sensor,Camera,Pusher Group
class if_cap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['true','false'])
        self.counter = 0
        self.list1 = ['true','false']

    def execute(self, userdata):
        rospy.loginfo('Cylinder Detected by Sensor')
        rospy.sleep(4)
        return random.choice(self.list1)


class Push(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Object Detection Initalized')
        rospy.sleep(2)
        return 'success'

class Pass(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Cylinder Released')
        rospy.sleep(2)
        return 'success'

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

    ## Pusher Group
    sm_pusher = smach.StateMachine(outcomes=[])
    with sm_pusher:
        smach.StateMachine.add('if_cap', if_cap(), 
                               transitions={'false':'Push',
                                            'true': 'Pass'})
        smach.StateMachine.add('Push', Push(), 
                               transitions={'success':'if_cap'})
        smach.StateMachine.add('Pass', Pass(), 
                               transitions={'success':'if_cap'})
    
    ## Stopper Group
    sm_stopper = smach.StateMachine(outcomes=[])
    stopper_sensor_topic_name = rospy.get_param('/stopper_sensor/topic_name')
    with sm_stopper:
        smach.StateMachine.add('STOP', smach_ros.MonitorState(stopper_sensor_topic_name, CountSensor, stopper_sensor_cb), transitions={'invalid':'STOP', 'valid':'Detect', 'preempted':'STOP'})

        smach.StateMachine.add('Detect', Detect(), 
                                transitions={'success':'Release'})
        smach.StateMachine.add('Release', Release(), 
                                transitions={'success':'Stop'})

    ## Concurrence of Pusher Group and Stopper Group
    sm_con = smach.Concurrence(outcomes=['succeeded','preempted','aborted'], default_outcome='succeeded')
    with sm_con:
        smach.Concurrence.add('PUSHER_GROUP' , sm_pusher)
        smach.Concurrence.add('STOPPER_GROUP',sm_stopper)

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    with sm_top:
        smach.StateMachine.add('RESET',sm_reset, transitions={'succeeded':'MAIN'})
        smach.StateMachine.add('MAIN',sm_con)
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(rospy.get_name(), sm_top, '/JETSON_SYS')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()