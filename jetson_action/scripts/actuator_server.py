#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from jetson_msgs.srv import Actuator,ActuatorResponse

def actuate_cb(req):
    rospy.sleep(delay_secs)
    GPIO(actuator_pin, req.actuate) 
    rospy.loginfo("%s action: %s"%(rospy.get_name(), req.actuate))
    return ActuatorResponse(True)

if __name__ == "__main__":
    try:
        rospy.init_node('actuator_server')
        topic_name = rospy.get_param('~topic_name')
        delay_secs = rospy.get_param('~delay_action')
        ## Initialize gpio
        actuator_pin = rospy.get_param('~gpio')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(actuator_pin,GPIO.OUT)
        ## Start Server
        rospy.Service(topic_name, Actuator, actuate_cb)
        rospy.loginfo("Actuator is ready!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()

