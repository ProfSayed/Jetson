#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from jetson_msgs.srv import Actuator,ActuatorResponse

def actuate_cb(req):
    GPIO.output(actuator_pin, req.actuate) 
    rospy.loginfo("%s Action: %s"%(rospy.get_name(), req.actuate))
    return ActuatorResponse(True)

if __name__ == "__main__":
    try:
        rospy.init_node('actuator_server')
        topic_name = rospy.get_param('~topic_name')
        ## Initialize gpio
        actuator_pin = rospy.get_param('~gpio')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(actuator_pin, GPIO.OUT, initial=GPIO.LOW)
        ## Start Server
        rospy.Service(topic_name, Actuator, actuate_cb)
        rospy.loginfo("Actuator is ready!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.output(actuator_pin, GPIO.LOW) 
        GPIO.cleanup()

