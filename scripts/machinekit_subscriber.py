#! /usr/bin/env python

import rospy
import hal
from std_msgs.msg import String, Float32

h = None

def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+ "\nI heard %s", data.data )
    h['value-out']=data.data
    print("the HAL pin \"rossub-outside-HAL.value-out\" just got updated to %s" ) % data.data


def listener():
    global h
    rospy.init_node('machinekit_subscriber', anonymous=True)
    rospy.Subscriber("hello_pub", Float32, callback)
    # Create a new hal userland component
    h = hal.component("rossub-outside-HAL")
    # Create a pin to read. A signal can be attached to this pin
    h.newpin("value-out", hal.HAL_FLOAT, hal.HAL_OUT)
    h.ready()
    rospy.spin()

if __name__ == '__main__':
    listener()
