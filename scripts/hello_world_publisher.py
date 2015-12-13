#! /usr/bin/env python

import rospy
import hal #import the puthon HAL bindings to connect to Machinekit
from std_msgs.msg import String, Float32

def talker():
    pub = rospy.Publisher('hello_pub', Float32, queue_size=10)
    rospy.init_node('hello_world_publisher', anonymous=True)
    r = rospy.Rate(10)
    # Create a new hal userland component
    h = hal.component("rospub-inside-HAL")
    # Create a pin to read. A signal can be attached to this pin
    h.newpin("value-in", hal.HAL_FLOAT, hal.HAL_IN)
    while not rospy.is_shutdown():
        number = h['value-in']
        str = "rospub-inside-HAL.value at %s is %s" % (rospy.get_time(),number)
        rospy.loginfo(str)
        str = "rospub-inside-HAL.value is : %s" % (number)
        pub.publish(number)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
        
