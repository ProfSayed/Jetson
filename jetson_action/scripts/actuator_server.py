#!/usr/bin/env python3
import rospy
import board
import digitalio
from jetson_msgs.srv import Actuator,ActuatorResponse

def actuate_cb(req):
    actuator_io.value = req.actuate
    rospy.loginfo("Stopping Action: %s"%(req.actuate))
    return ActuatorResponse(True)

if __name__ == "__main__":
    rospy.init_node('actuator_server')
    topic_name = rospy.get_param('~topic_name')
    gpio = rospy.get_param('~gpio')
    ## Initialize gpio
    actuator_pin = gpio
    actuator_io = digitalio.DigitalInOut(actuator_pin)
    actuator_io.direction = digitalio.Direction.OUTPUT
    ## Start Server
    rospy.Service(topic_name, Actuator, actuate_cb)
    rospy.loginfo("Actuator is ready!")
    rospy.spin()
