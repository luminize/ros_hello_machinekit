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

servothread = "servo_thread"
card = "hm2_5i20.0"
rt.newthread('%s' % servothread, 1000000, fp=True)
hal.addf('%s.read' % card, servothread)
hal.addf('%s.write' % card, servothread)
hal.addf('%s.pet_watchdog' % card, servothread)

# set up the stepgens
for i in range(0, 4):
    # these are in nanoseconds
    SCALE      =        1000
    DIRSETUP   =        200
    DIRHOLD    =        200
    STEPLEN    =        40000
    STEPSPACE  =        40000
    STEPGEN_MAX_VEL =   12
    STEPGEN_MAX_ACC =   24

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
