#!/usr/bin/python

import rtm
from rtm import *
rtm.nsport = 2809
rtm.nshost = "localhost"

import OpenHRP
from OpenHRP import *

import HRP5P

import cnoid.Corba
orb = cnoid.Corba.getORB()
rtm.orb = orb
nsloc = "corbaloc:iiop:%s:%d/NameService" % (rtm.nshost, rtm.nsport)
print nsloc
rtm.rootnc = orb.string_to_object(nsloc)._narrow(CosNaming.NamingContext)

import cnoid.Corba
from cnoid.grxui import *

import math


##################


def isEnabledHmc(mode=0):

    rpc_svc = narrow(hmc.service("RTCOutPortCreatorService","RTCOutPortCreatorService"),"RTCOutPortCreatorService")
    if rpc_svc != None:
        rpc_svc.isEnabled(mode)
    else:
        print "RTCOutPortCreatorService is not found in hmc"

def goActual():
    
    sh_svc.goActual()
        
def goHalfSitting(_halfConf, _halfConfWaistHeight, tm=3.0, wait=True):

    seq_svc.setJointAngles(_halfConf, tm)
    waistpos = [0,0,_halfConfWaistHeight]
    if not seq_svc.setBasePos(waistpos, tm):
        print "setBasePos() failed"
    zmp = [0,0,-_halfConfWaistHeight]
    if not seq_svc.setZmp(zmp, tm):
        print "setZmp() failed"
    if wait==True:
        seq_svc.waitInterpolation()
    print "finished go-half sitting"

def goHalfSittingHmc(_halfConf, _halfConfWaistHeight, tm=3.0):
    
    isEnabledHmc(0)
    goActual()
    goHalfSitting(_halfConf, _halfConfWaistHeight, tm, True)

        
##################


def createComps():

    global rh, servo, ms, seq, seq_svc, sh, sh_svc, kf, hmc, hmc_svc

    rh = rtm.findRTC("HRP5PController(Robot)0")
    servo = rtm.findRTC("ModifiedServo0")

    ms = rtm.findRTCmanager()

    ms.load("KalmanFilter")
    kf = ms.create("KalmanFilter")
    kf.setProperty("Qbw", "0.0001")
    kf.setProperty("filter_order", "3")
    kf.setProperty("Tgsens", "0.15")

    ms.load("StateHolder")
    sh = ms.create("StateHolder")
    sh_svc = narrow(sh.service("service0"), "StateHolderService")
    
    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    seq_svc = narrow(seq.service("service0"), "SequencePlayerService")

    ms.load("HumanoidMotionControl2")
    hmc = ms.create("HumanoidMotionControl2", "hmc")
    hmc_svc = narrow(hmc.service("service0"), "HumanoidMotionControlService")

    hmc.setProperty("debug-level", "3")
    hmc.setProperty("initialGroup", "")
    hmc.setProperty("plugins", "LiftFoot_IROS2018.yaml")

def connectOfflineComps():

    connectPorts(rh.port("gyrometer"),  kf.port("rate"))
    connectPorts(rh.port("gsensor"),    kf.port("acc"))

    connectPorts(rh.port("q"),          sh.port("currentQIn"))
    connectPorts(sh.port("qOut"),       servo.port("qRef"))

    connectPorts(sh.port("qOut"),       seq.port("qInit"))
    connectPorts(sh.port("basePosOut"), seq.port("basePosInit"))
    connectPorts(sh.port("baseRpyOut"), seq.port("baseRpyInit"))
    
    connectPorts(seq.port("qRef"),      sh.port("qIn"))
    connectPorts(seq.port("basePos"),   sh.port("basePosIn"))
    connectPorts(seq.port("baseRpy"),   sh.port("baseRpyIn"))

def connectOnlineComps():

    connectPorts(sh.port("qOut"),              hmc.port("qInit"))
    connectPorts(rh.port("q"),                 hmc.port("q"))

    connectPorts(rh.port("waistAbsTransform"), hmc.port("waistAbsTransform"))
    connectPorts(rh.port("waistAbsVelocity"),  hmc.port("waistAbsVelocity"))
    
    connectPorts(kf.port("rpy"),               hmc.port("rpy"))
    connectPorts(rh.port("gyrometer"),         hmc.port("rate"))
    connectPorts(rh.port("gsensor"),           hmc.port("acc"))
    
    connectPorts(rh.port("rfsensor"),          hmc.port("rfforce"))
    connectPorts(rh.port("lfsensor"),          hmc.port("lfforce"))
    connectPorts(rh.port("rhsensor"),          hmc.port("rhforce"))
    connectPorts(rh.port("lhsensor"),          hmc.port("lhforce"))
    
    connectPorts(hmc.port("qRef"),             sh.port("qIn"))
    connectPorts(hmc.port("tauRef"),           servo.port("tauRef"))
    connectPorts(hmc.port("torqueMode"),       servo.port("torqueMode"))

def activateComps():

    rtm.serializeComponents([rh, kf, sh, seq, hmc])

    rh.start()
    kf.start()
    sh.start()
    seq.start()
    hmc.start(None, 30.0)
    
def init():
    
    print "creating components"
    createComps()

    print "connecting offline components"
    connectOfflineComps()

    print "activating components"
    activateComps()

    print "connecting online components"
    connectOnlineComps()

    goHalfSittingHmc(HRP5P.halfConf, HRP5P.halfConfWaistHeight, 1.0)
    
    isEnabledHmc(1)
    hmc_svc.sendMsg("multi-contact-motion-solver :enable-solver 1")

    #time.sleep(0.005)
    feedback_torque_mode()

    print "initialized successfully"


##################


def startHumanoidMotionControlPlugins():
    
    try:
        hmc_svc.sendMsg("hmc :start-group-wait HumanoidMotionControl")
        startLog()
    except:
        print "Cannot start HumanoidMotionControl"

def startArmsMotionControlPlugins():

    try:
        hmc_svc.sendMsg("hmc :start-group-wait ArmsMotionControl")
    except:
        print "Cannot start ArmsMotionControl"

def holdRightFoot():

    hmc_svc.sendMsg("rfoot-sole-contact :start")
    hmc_svc.sendMsg("rfoot-pose-task    :stop")

def holdLeftFoot():
    
    hmc_svc.sendMsg("lfoot-sole-contact :start")
    hmc_svc.sendMsg("lfoot-pose-task    :stop")

def releaseRightFoot():

    hmc_svc.sendMsg("rfoot-pose-task    :start")
    hmc_svc.sendMsg("rfoot-sole-contact :stop")

def releaseLeftFoot():

    hmc_svc.sendMsg("lfoot-pose-task    :start")
    hmc_svc.sendMsg("lfoot-sole-contact :stop")

def startBodyComTrajectoryGenerationPlugins():

    try:
        hmc_svc.sendMsg("hmc :start-group-wait BodyComTrajectoryGeneration")
    except:
        print "cannot start BodyComTrajectoryGeneration"

def startHandsTrajectoryGenerationPlugins():

    try:
        hmc_svc.sendMsg("hmc :start-group-wait RightHandTrajectoryGeneration")
    except:
        print "cannot start RightHandTrajectoryGeneration"

    try:
        hmc_svc.sendMsg("hmc :start-group-wait LeftHandTrajectoryGeneration")
    except:
        print "cannot start LeftHandTrajectoryGeneration"

def startRightFootTrajectoryGenerationPlugins():

    try:
        hmc_svc.sendMsg("hmc :start-group-wait RightFootTrajectoryGeneration")
    except:
        print "cannot start RightFootTrajectoryGeneration"

def stopRightFootTrajectoryGenerationPlugins():

    try:
        hmc_svc.sendMsg("hmc :stop-group RightFootTrajectoryGeneration")
    except:
        print "cannot stop RightFootTrajectoryGeneration"
        
def startLeftFootTrajectoryGenerationPlugins():
        
    try:
        hmc_svc.sendMsg("hmc :start-group-wait LeftFootTrajectoryGeneration")
    except:
        print "cannot start LeftFootTrajectoryGeneration"

def stopLeftFootTrajectoryGenerationPlugins():
        
    try:
        hmc_svc.sendMsg("hmc :stop-group-wait LeftFootTrajectoryGeneration")
    except:
        print "cannot stop LeftFootTrajectoryGeneration"
        
def startLog():

    try:
        hmc_svc.sendMsg("hmc :start-group-wait Logger")
        print "Logger started"
    except:
        print "Cannot start Logger"

def stopLog():

    try:
        hmc_svc.sendMsg("hmc :stop-group-wait Logger")
        print "Logger stopped"
    except:
        print "Cannot stop Logger"
        
def saveLog():

    try:
        stopLog()
        base_name = "HMCLog/LiftFoot_state"
        hmc_svc.sendMsg("state-logger :save " + base_name)
        startLog()
        print "Log saved"
    except:
        print "Cannot save log"

def feedback_torque_mode():

    hmc_svc.sendMsg("multi-contact-motion-solver :enable-feedback-torque-mode 1")

def reset_body():

    hmc_svc.sendMsg("posture-task :reset")
    hmc_svc.sendMsg("com-task :reset")
    hmc_svc.sendMsg("body-pose-task :reset")
    hmc_svc.sendMsg("chest-orientation-task :reset")

def reset_hands():
    
    hmc_svc.sendMsg("rhand-pose-task :reset")
    hmc_svc.sendMsg("lhand-pose-task :reset")

def reset_feet():

    hmc_svc.sendMsg("rfoot-pose-task :reset")
    hmc_svc.sendMsg("lfoot-pose-task :reset")

##################


def set_comxy_pz(x, y, z, T):

    hmc_svc.sendMsg("body-position-trajectory-generator :configure-timing " + str(T))
    hmc_svc.sendMsg("body-position-trajectory-generator :set-positions 0.0 0.0 " + str(z))

    hmc_svc.sendMsg("com-trajectory-generator           :configure-timing " + str(T))
    hmc_svc.sendMsg("com-trajectory-generator           :set-positions " + str(x) + " " + str(y) + " 0.0")

def set_hands_pose(x, y, z, rolld, pitchd, yawd, T):
    
    hmc_svc.sendMsg("rhand-position-trajectory-generator    :configure-timing " + str(T))
    hmc_svc.sendMsg("rhand-position-trajectory-generator    :set-positions " + str(x) + " " + str(-y) + " " + str(z))
    
    hmc_svc.sendMsg("rhand-orientation-trajectory-generator :configure-timing " + str(T))
    hmc_svc.sendMsg("rhand-orientation-trajectory-generator :set-orientations " + str(rolld) + " " + str(pitchd) + " " + str(yawd))

    hmc_svc.sendMsg("lhand-position-trajectory-generator    :configure-timing " + str(T))
    hmc_svc.sendMsg("lhand-position-trajectory-generator    :set-positions " + str(x) + " " + str(y) + " " + str(z))

    hmc_svc.sendMsg("lhand-orientation-trajectory-generator :configure-timing " + str(T))
    hmc_svc.sendMsg("lhand-orientation-trajectory-generator :set-orientations " + str(-rolld) + " " + str(pitchd) + " " + str(-yawd))

def set_right_foot_pose(x, y, z, rolld, pitchd, yawd, T):
    
    hmc_svc.sendMsg("rfoot-position-trajectory-generator    :configure-timing " + str(T))
    hmc_svc.sendMsg("rfoot-position-trajectory-generator    :set-positions " + str(x) + " " + str(y) + " " + str(z))
    
    hmc_svc.sendMsg("rfoot-orientation-trajectory-generator :configure-timing " + str(T))
    hmc_svc.sendMsg("rfoot-orientation-trajectory-generator :set-orientations " + str(rolld) + " " + str(pitchd) + " " + str(yawd))

def set_left_foot_pose(x, y, z, rolld, pitchd, yawd, T):
    
    hmc_svc.sendMsg("lfoot-position-trajectory-generator    :configure-timing " + str(T))
    hmc_svc.sendMsg("lfoot-position-trajectory-generator    :set-positions " + str(x) + " " + str(y) + " " + str(z))

    hmc_svc.sendMsg("lfoot-orientation-trajectory-generator :configure-timing " + str(T))
    hmc_svc.sendMsg("lfoot-orientation-trajectory-generator :set-orientations " + str(rolld) + " " + str(pitchd) + " " + str(yawd))
    
    
##################


def halfsitting():

    set_comxy_pz(0, 0, 0.7, 1)
    set_hands_pose(0.0, 0.4, 0.8, 0, 45, 0, 1)

def rise_hands():

    set_hands_pose(0.25, 0.5, 1.5, 0, 90, 0, 1)

def com_over_rf():

    set_comxy_pz(0, -0.095, 0.7, 1)

def com_over_lf():

    set_comxy_pz(0,  0.095, 0.7, 1)

def rf_support():

    startLeftFootTrajectoryGenerationPlugins()
    releaseLeftFoot()
    
def lf_support():

    startRightFootTrajectoryGenerationPlugins()
    releaseRightFoot()
    
def double_support_from_rf():

    holdLeftFoot()
    stopLeftFootTrajectoryGenerationPlugins()
    
def double_support_from_lf():

    holdRightFoot()
    stopRightFootTrajectoryGenerationPlugins()
    
def lift_rf():
    
    set_right_foot_pose(0.0, -0.095, 0.25, 0, 0, 0, 1)

def land_rf():
    
    set_right_foot_pose(0.0, -0.095, 0.1,  0, 0, 0, 1)

def lift_lf():
    
    set_left_foot_pose(0.0, 0.095, 0.25, 0, 0, 0, 1)

def land_lf():
    
    set_left_foot_pose(0.0, 0.095, 0.1,  0, 0, 0, 1)

    
##################


MenuList = [
    ['--------- Utility ---------', '#label',
     'Go Standard Half-Sitting', 'goHalfSittingHmc(HRP5P.halfConf, HRP5P.halfConfWaistHeight, 1.0)',
     'Save log', 'saveLog()'],
    ['Halfsitting',    'halfsitting()',
     'Rise hands',     'rise_hands()',
     'Com over RF',    'com_over_rf()',
     'Support by RF',  'rf_support()',
     'Lift LF',        'lift_lf()',
     'Land LF',        'land_lf()',
     'Double support', 'double_support_from_rf()',
     'Com over LF',    'com_over_lf()',
     'Support by LF',  'lf_support()',
     'Lift RF',        'lift_rf()',
     'Land RF',        'land_rf()',
     'Double support', 'double_support_from_lf()']
    ]

    
##################


init()

startHumanoidMotionControlPlugins()
startArmsMotionControlPlugins()
startBodyComTrajectoryGenerationPlugins()
startHandsTrajectoryGenerationPlugins()
    
waitInputMenu(MenuList)
