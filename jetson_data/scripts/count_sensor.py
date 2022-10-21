#!/usr/bin/env python3
import rospy
import board
from jetson_msgs.msg import CountSensor
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

def main():
    rospy.init_node('Sensor', anonymous=True)
    ## Get Parameters
    topic_name = rospy.get_param('~topic_name')
    i2c_address = rospy.get_param("~i2c_address")
    index = rospy.get_param('/initial_cylinder_index')
    current_thresh = rospy.get_param('/sensor_current_threshold')
    freq_hz = rospy.get_param('/frequency')

    ## GPIO
    i2c_bus = board.I2C()
    ina219 = INA219(i2c_bus, i2c_address)
    ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
    ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
    ina219.bus_voltage_range = BusVoltageRange.RANGE_16V
    ## On detection of a new cylinder => True
    new_cylinder = False

    pub = rospy.Publisher(topic_name, CountSensor, queue_size=10)
    rate = rospy.Rate(freq_hz) 
    msg = CountSensor()
    
    while not rospy.is_shutdown():
        ## Count Cylinders
        current = ina219.current
        if current < current_thresh:
            new_cylinder = True

        if current > current_thresh:
            if new_cylinder:
                new_cylinder = False
                index += 1
            ## Publish Message
            msg.time_stamp = rospy.Time.now()
            msg.cylinder_number = index
            pub.publish(msg)
        
        ## Check for Overflow
        if ina219.overflow:
            rospy.logwarn("Internal Math Overflow Detected!")

        rate.sleep()

if __name__ == '__main__':
    try:       
        main()
    except rospy.ROSInterruptException:
        pass