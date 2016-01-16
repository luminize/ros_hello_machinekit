import os
import sys
from machinekit import hal
from machinekit import rtapi as rt
from machinekit import config as c

hardwarelist = {
    'mesa-5i20' : ['mesanet 5i20 anything IO FPGA card','hm2_5i20.0'],
    'bbb-cramps' : ['BeagleBone Black (PRU) with CRAMPS cape', 'hpg']
}

if len(sys.argv) == 2:
    # get the hardware type from the command line
    hardware = str(sys.argv[1])
else:
    print("usage: %s <hardware>" % sys.argv[0])
    exit(1)

if not hardware in hardwarelist:
    print("USAGE: %s <hardware>" % sys.argv[0])
    print("Argument \"%s\" is not in hardware list") % hardware
    print("")
    print "{:<15} {:<20}".format('Argument','Description')
    print("----------------------------------------------")
    for argument, description in hardwarelist.iteritems():
        print "{:<15} {:<20}".format(argument, description[0])
    exit(1)
else:
    print("card description  : %s, %s") % (hardware, hardwarelist[hardware][0])
    print("hal component name: %s") % (hardwarelist[hardware][1])

# try to connect to HAL
rt.init_RTAPI()

# set the correct string for HAL component, depending on the hardware
card = hardwarelist[hardware][1]

#load mesa card with firmware
if hardware == 'mesa-5i20':
    rt.loadrt('hostmot2')
    rt.loadrt('hm2_pci', config="firmware=hm2/5i20/SVST8_4.BIT \
                                num_pwmgens=3 \
                                num_stepgens=4")
if hardware == 'bbb-cramps':
    os.system('./setup.sh')
    rt.loadrt('hal_bb_gpio',
              output_pins='816,822,823,824,825,826,914,923,925',
              input_pins='807,808,809,810,817,911,913')
    prubin = '%s/%s' % (c.Config().EMC2_RTLIB_DIR, 'xenomai/pru_generic.bin')
    rt.loadrt('hal_pru_generic',
              pru=0, num_stepgens=4,
              num_pwmgens=0,
              prucode=prubin,
              halname=card)


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

# create a new thread
rt.newthread('%s' % servothread, 1000000, fp=True)

# add components to the thread

if hardware == 'mesa-5i20':
    hal.addf('%s.read' % card, servothread)
    hal.addf('siggen.0.update', servothread)
    hal.addf('joint_stream', servothread)
    hal.addf('%s.write' % card, servothread)
    hal.addf('%s.pet_watchdog' % card, servothread)
    
if hardware == 'bbb-cramps':
    hal.addf('%s.capture-position' % card, servothread)
    hal.addf('bb_gpio.read', servothread)
    hal.addf('siggen.0.update', servothread)
    hal.addf('joint_stream', servothread)
    hal.addf('%s.update' % card, servothread)
    hal.addf('bb_gpio.write', servothread)

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
    
    # hpg doesn't have this pin
    if not hardware == 'bbb-cramps':
        hal.Pin('%s.stepgen.0%s.step_type' % (card, i)).set(0)

    signal = hal.newsig('emcmot.0%s.enable' % i, hal.HAL_BIT)
    signal.link('%s.stepgen.0%s.enable' % (card, i))
    signal.set(1)

# link joint_stream.joint.00.pos-cmd to the stepgen.00.position-cmd
for i in range(0, 4):
    sig_joint = hal.newsig('sig-joint%s' % i, hal.HAL_FLOAT)
    sig_joint.link('joint_stream.joint.0%s.pos-cmd' % i)
    sig_joint.link('%s.stepgen.0%s.position-cmd' % (card,i) )

# some more BBB cramps specific    
if hardware == 'bbb-cramps':
    hal.Pin('%s.stepgen.00.steppin' % card).set(813)
    hal.Pin('%s.stepgen.00.dirpin' % card).set(812)
    hal.Pin('%s.stepgen.01.steppin' % card).set(815)
    hal.Pin('%s.stepgen.01.dirpin' % card).set(814)
    hal.Pin('%s.stepgen.02.steppin' % card).set(819)
    hal.Pin('%s.stepgen.02.dirpin' % card).set(818)
    hal.Pin('%s.stepgen.03.steppin' % card).set(916)
    hal.Pin('%s.stepgen.03.dirpin' % card).set(911)
    # Machine power
    hal.net('emcmot.00.enable', 'bb_gpio.p9.out-23')
    # Tie machine power signal to the CRAMPS LED
    hal.net('emcmot.00.enable', 'bb_gpio.p9.out-25')
    # set some better steps for video
    for i in range(0, 4):
        hal.Pin('%s.stepgen.0%s.position-scale' % (card, i)).set(4000)
