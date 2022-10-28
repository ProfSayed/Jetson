#!/usr/bin/env python3
import rospy
import board
import RPi.GPIO as GPIO
import digitalio
from jetson_msgs.srv import Actuator,ActuatorResponse

def actuate_cb(req):
    actuator_io.value = req.actuate
    rospy.loginfo("%s action: %s"%(rospy.get_name(), req.actuate))
    return ActuatorResponse(True)

if __name__ == "__main__":
    try:
        rospy.init_node('actuator_server')
        topic_name = rospy.get_param('~topic_name')
        ## Initialize gpio
        actuator_pin = rospy.get_param('~gpio')
        if actuator_pin == 17:
            actuator_io = digitalio.DigitalInOut(board.D17)
        if actuator_pin == 22:
            actuator_io = digitalio.DigitalInOut(board.D22)
        actuator_io.direction = digitalio.Direction.OUTPUT
        ## Start Server
        rospy.Service(topic_name, Actuator, actuate_cb)
        rospy.loginfo("Actuator is ready!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        actuator_io.value = False
        GPIO.cleanup()
