#! /usr/bin/env python
from machinekit import hal

import rospy
from sensor_msgs.msg import JointState

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

c = None

def get_topic_type(topic_name):
  import rostopic
  (msgtype, _, _) = rostopic.get_topic_class(topic_name, blocking=True)
  return msgtype

def gen_halcomp(cname, prefix, msgtype, dir, count):
  ''' define a HAL component whose pins match msg field names and types '''
  global c
  c = hal.Component(cname)
  msg = msgtype()
  pins = dict()
  for i in range(len(msg.__slots__)):
    fname = msg.__slots__[i]
    ftype = msg._slot_types[i]
    pinlist = list()
    if ftype in arraytypes:

      for i in range(count):
        pname = "%s.%s.%d" % (prefix, fname, i)
        pinlist.append(c.newpin(pname, typemap[arraytypes[ftype]], dir))
      pins[fname] = pinlist
      continue

    if not ftype in typemap:
      print("ignoring type: %s") % (ftype)
      pins[fname] = None
    else:
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
  for item, (field, obj) in enumerate(pins.iteritems()):
    #output type and field for debug
    print("item: %s type: %s value: %s") % (item, obj, field)
    if obj is not None:
        if type(obj) is list:
          values = msg.__getattribute__(field)
          print(c)
          if values:
              for j in range(len(obj)):
                  print(values[j])
                  obj[j].set(values[j])
        else:
          values = msg.__getattribute__(field)
          obj.set(msg.__getattribute__(field))

def demo_subscriber(compname, prefix, topic, count=6):
  '''
  retrieve msg type of <topic>
  subscribe to <topic>
  create pins <compname>.<prefix>.fieldname of appropriate type
  update pin values accordingly
  '''
  global pins
  msgtype = get_topic_type(topic)
  pins = gen_halcomp(compname, prefix, msgtype, hal.HAL_OUT, count)
  # create signal from joint_states.robot.position.0 to hm2_5i20.0.stepgen.00.position-cmd
  # net joint0 joint_states.robot.position.0 hm2_5i20.0.stepgen.00.position-cmd
  for i in range(0, 2):
      sig_joint = hal.newsig('sig-joint%s' % i, hal.HAL_FLOAT)
      sig_joint.link('joint_states.robot.position.%s' % i)
      sig_joint.link('hm2_5i20.0.stepgen.0%s.position-cmd' % i)

  rospy.init_node(compname, anonymous=True)
  rospy.Subscriber(topic, msgtype, callback=demo_callback, callback_args=pins)
  rospy.spin()


def demo_publisher(compname, prefix, msgtype, count=3):
  pins = gen_halcomp(compname, prefix, msgtype, hal.HAL_IN, count)
  try:
    talker(pins, compname, prefix, msgtype, 10)
  except rospy.ROSInterruptException: pass

if __name__ == '__main__':

#  demo_publisher('test', 'jtp', JointTrajectoryPoint, count=3)

  demo_subscriber("joint_states",
                "robot",
                "/joint_states")
