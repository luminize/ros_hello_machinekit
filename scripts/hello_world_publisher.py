#! /usr/bin/env python

import rospy
import hal
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('hello_pub', String, queue_size=10)
    rospy.init_node('hello_world_publisher', anonymous=True)
    r = rospy.Rate(10)
    h = hal.component("rospub-inside-HAL")
    h.newpin("value-in", hal.HAL_S32, hal.HAL_IN)
    while not rospy.is_shutdown():
        number = h['value-in']
        str = "rospub-inside-HAL.value at %s is %s" % (rospy.get_time(),number)
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
        
