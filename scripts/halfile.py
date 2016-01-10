import os
from machinekit import hal
from machinekit import rtapi as rt


# try to connect to HAL
rt.init_RTAPI()

#load mesa card with firmware
rt.loadrt('hostmot2')
rt.loadrt('hm2_pci', config="firmware=hm2/5i20/SVST8_4.BIT \
                            num_pwmgens=3 \
                            num_stepgens=4")

rt.loadrt('siggen')
#rt.newinst("siggen","siggen_0")

# load the ring for use with Charles' joint_stream component.
# for this you need to have built with catkin_make
# https://github.com/cdsteinkuehler/ros_hal_tests.git
# and also installed the component `comp --install joint_stream.comp`

# newring jointpos 16384
r1 = hal.Ring("jointpos", size=16384)

rt.loadrt("joint_stream","ring=jointpos")

# create servo_thread
servothread = "servo_thread"
card = "hm2_5i20.0"
rt.newthread('%s' % servothread, 1000000, fp=True)

# add components to the thread
hal.addf('%s.read' % card, servothread)
hal.addf('%s.write' % card, servothread)
hal.addf('%s.pet_watchdog' % card, servothread)
hal.addf('siggen.0.update', servothread)
hal.addf('joint_stream', servothread)

# set up the stepgens
for i in range(0, 4):
    # these are in nanoseconds
    DIRSETUP   =        200
    DIRHOLD    =        200
    STEPLEN    =        40000
    STEPSPACE  =        40000
    STEPGEN_MAX_VEL =   100000
    STEPGEN_MAX_ACC =   100000
    # 16 microstepping means 3200 steps/receive
    # radians as input: rad -> rev => rad/(2*PI)
    SCALE      =        3200 / (2 * 3.1415692)

    hal.Pin('%s.stepgen.0%s.dirsetup' % (card, i)).set(DIRSETUP)
    hal.Pin('%s.stepgen.0%s.dirhold' % (card, i)).set(DIRHOLD)
    hal.Pin('%s.stepgen.0%s.steplen' % (card, i)).set(STEPLEN)
    hal.Pin('%s.stepgen.0%s.stepspace' % (card, i)).set(STEPSPACE)
    hal.Pin('%s.stepgen.0%s.position-scale' % (card, i)).set(SCALE)
    hal.Pin('%s.stepgen.0%s.maxvel' % (card, i)).set(STEPGEN_MAX_VEL)
    hal.Pin('%s.stepgen.0%s.maxaccel' % (card, i)).set(STEPGEN_MAX_ACC)
    hal.Pin('%s.stepgen.0%s.step_type' % (card, i)).set(0)

    signal = hal.newsig('emcmot.0%s.enable' % i, hal.HAL_BIT)
    signal.link('%s.stepgen.0%s.enable' % (card, i))
    signal.set(1)

# link joint_stream.joint.00.pos-cmd to the stepgen.00.position-cmd
for i in range(0, 2):
    sig_joint = hal.newsig('sig-joint%s' % i, hal.HAL_FLOAT)
    sig_joint.link('joint_stream.joint.0%s.pos-cmd' % i)
    sig_joint.link('hm2_5i20.0.stepgen.0%s.position-cmd' % i)
