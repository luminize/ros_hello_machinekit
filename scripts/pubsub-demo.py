#! /usr/bin/env python
from machinekit import hal

import rospy
from turtlesim.msg import Pose
#from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint

# maps msg types to HAL types
typemap = {
  'float32' : hal.HAL_FLOAT,
  'float64' : hal.HAL_FLOAT,
  'int32' :   hal.HAL_S32,
  'uint32' :  hal.HAL_U32,
  'bool' :    hal.HAL_BIT,
  'duration' :    hal.HAL_S32,
}


arraytypes = {
    'float32[]' : 'float32',
    'float64[]' : 'float64',
    'int32[]' :   'int32',
    'uint32[]' :  'uint32'
}


def get_topic_type(topic_name):
  import rostopic
  (msgtype, _, _) = rostopic.get_topic_class(topic_name, blocking=True)
  return msgtype

def gen_halcomp(cname, prefix, msgtype, dir, count):
  ''' define a HAL component whose pins match msg field names and types '''
  c = hal.Component(cname)
  msg = msgtype()
  pins = dict()
  for i in range(len(msg.__slots__)):
    fname = msg.__slots__[i]
    ftype = msg._slot_types[i]

    if ftype in arraytypes:
      pinlist = list()
      for i in range(count):
        pname = "%s.%s.%d" % (prefix, fname,i)
        pinlist.append(c.newpin(pname, typemap[arraytypes[ftype]], dir))
      pins[fname] = pinlist
      continue

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
    for field, obj in pins.iteritems():
      if type(obj) is list:
        values = list()
        for pin in obj:
          values.append(pin.get())
        msg.__setattr__(field, values)
      else:
        msg.__setattr__(field, obj.get())
    pub.publish(msg)
    r.sleep()


def demo_callback(msg, pins):
  # rospy.loginfo(rospy.get_caller_id()+ "\nI heard %s", msg )
  for field, obj in pins.iteritems():
    if type(obj) is list:
      values = msg.__getattribute__(field)
      for pin in obj:
        pin.set(values.pop())
    else:
      obj.set(msg.__getattribute__(field))


def demo_subscriber(compname, prefix, topic, count=3):
  '''
  retrieve msg type of <topic>
  subscribe to <topic>
  create pins <compname>.<prefix>.fieldname of appropriate type
  update pin values accordingly
  '''
  msgtype = get_topic_type(topic)
  pins = gen_halcomp(compname, prefix, msgtype, hal.HAL_OUT, count)
  rospy.init_node(compname, anonymous=True)
  rospy.Subscriber(topic, msgtype, callback=demo_callback, callback_args=pins)
  rospy.spin()


def demo_publisher(compname, prefix, msgtype, count=3):
  pins = gen_halcomp(compname, prefix, msgtype, hal.HAL_IN, count)
  try:
    talker(pins, compname, prefix, msgtype, 10)
  except rospy.ROSInterruptException: pass

if __name__ == '__main__':

  demo_publisher('test', 'jtp', JointTrajectoryPoint, count=3)

  # mirrors the turtlesim demo pose as HAL pins:
  #demo_subscriber("turtle1", "pose", "/turtle1/pose")

