#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Float32

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+ "\nI heard %s", data.data )


def listener():
    rospy.init_node('hello_world_subscriber', anonymous=True)
    rospy.Subscriber("hello_pub", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
