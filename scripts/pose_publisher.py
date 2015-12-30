#! /usr/bin/env python

import rospy
from turtlesim.msg import Pose
from machinekit import hal

cname = 'halturtle'
topic = 'pose'

# maps msg types to HAL types
typemap = {
    'float32' : hal.HAL_FLOAT,
    'float64' : hal.HAL_FLOAT,
    'int32' :   hal.HAL_S32,
    'uint32' :  hal.HAL_U32,
    'bool' :    hal.HAL_BIT,
}

def gen_halcomp(name, topic, msgtype, dir ):
    ''' define a HAL component whose pins match msg field names and types '''
    c = hal.Component(name)
    msg = msgtype()
    pins = dict()
    for i in range(len(msg.__slots__)):
        fname = msg.__slots__[i]
        ftype = msg._slot_types[i]
        if not ftype in typemap:
            raise Exception,"type %s not supported" % ftype
        pname = "%s.%s" % (topic, fname)
        pins[fname] = c.newpin(pname, typemap[ftype], dir)
    c.ready()
    return pins

def talker(pins, cname, topic, msgtype, rate):
    ''' publish the pins in comp '''
    rospy.init_node(cname, anonymous=True)
    pub = rospy.Publisher(cname + "/" + topic, msgtype, queue_size=10)
    r = rospy.Rate(rate)
    msg = msgtype()
    while not rospy.is_shutdown():
        for i in range(len(msg.__slots__)):
            fname = msg.__slots__[i]
            msg.__setattr__(fname, pins[fname].get())
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    pins = gen_halcomp(cname, topic, Pose, hal.HAL_IN)
    try:
        talker(pins, cname, topic, Pose, 10)
    except rospy.ROSInterruptException: pass

