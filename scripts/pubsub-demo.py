#! /usr/bin/env python
from machinekit import hal

import rospy
from turtlesim.msg import Pose
#from geometry_msgs.msg import Twist


# maps msg types to HAL types
typemap = {
  'float32' : hal.HAL_FLOAT,
  'float64' : hal.HAL_FLOAT,
  'int32' :   hal.HAL_S32,
  'uint32' :  hal.HAL_U32,
  'bool' :    hal.HAL_BIT,
}

def get_topic_type(topic_name):
  import rostopic
  (msgtype, _, _) = rostopic.get_topic_class(topic_name, blocking=True)
  return msgtype

def gen_halcomp(cname, prefix, msgtype, dir ):
  ''' define a HAL component whose pins match msg field names and types '''
  c = hal.Component(cname)
  msg = msgtype()
  pins = dict()
  for i in range(len(msg.__slots__)):
    fname = msg.__slots__[i]
    ftype = msg._slot_types[i]
    if not ftype in typemap:
      raise Exception,"type %s not supported" % ftype
    pname = "%s.%s" % (prefix, fname)
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
    for field, pinobj in pins.iteritems():
      msg.__setattr__(field, pinobj.get())
    pub.publish(msg)
    r.sleep()


def demo_callback(msg, pins):
  # rospy.loginfo(rospy.get_caller_id()+ "\nI heard %s", msg )
  for field, pinobj in pins.iteritems():
    pinobj.set(msg.__getattribute__(field))


def demo_subscriber(compname, prefix, topic):
  '''
  retrieve msg type of <topic>
  subscribe to <topic>
  create pins <compname>.<prefix>.fieldname of appropriate type
  update pin values accordingly
  '''
  msgtype = get_topic_type(topic)
  pins = gen_halcomp(compname, prefix, msgtype, hal.HAL_OUT)
  rospy.init_node(compname, anonymous=True)
  rospy.Subscriber(topic, msgtype, callback=demo_callback, callback_args=pins)
  rospy.spin()


def demo_publisher(compname, prefix, msgtype):
  pins = gen_halcomp(compname, prefix, msgtype, hal.HAL_IN)
  try:
    talker(pins, compname, prefix, msgtype, 10)
  except rospy.ROSInterruptException: pass

if __name__ == '__main__':

  # unfortunately cant handle the Twist message yet (actionlib)
  # demo_publisher('turtle1', 'some_pose', Pose)

  # mirrors the turtlesim demo pose as HAL pins:
  demo_subscriber("turtle1", "pose", "/turtle1/pose")

