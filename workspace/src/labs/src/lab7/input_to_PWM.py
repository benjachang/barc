#!/usr/bin/env python

# Author: Tony Zheng
# MEC231A BARC Project 

import rospy
import time
from geometry_msgs.msg import Twist
from barc.msg import ECU, Input, Moving, Encoder 

# from encoder
v_meas = 0.0
t0 = time.time()
ang_km1     = 0.0
ang_km2     = 0.0
n_FL        = 0.0
n_FR        = 0.0
n_BL        = 0.0
n_BR        = 0.0
r_tire      = 0.05 # radius of the tire

# pwm cmds
motor_pwm   = 1500.0
motor_pwm_offset = 1550.0

# encoder measurement update
def enc_callback(data):
    global t0, v_meas
    global n_FL, n_FR, n_BL, n_BR
    global ang_km1, ang_km2

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute the average encoder measurement
    """
    CHANGE THIS DEPENDING ON WHICH ENCODERS WORK
    """
    n_mean = (n_FL + n_FR + n_BL)/3

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8

    # compute time elapsed
    tf = time.time()
    dt = tf - t0
    
    # compute speed with second-order, backwards-finite-difference estimate
    v_meas    = r_tire*(3*ang_mean - 4*ang_km1 + ang_km2)/(2*dt)
    # rospy.logwarn("speed = {}".format(v_meas))

    # update old data
    ang_km1 = ang_mean
    ang_km2 = ang_km1
    t0      = time.time()

def start_callback(data):
    global move, still_moving
    #print("2")
    #print(move)
    if data.linear.x >0:
        move = True
    elif data.linear.x <0:
        move = False
    #print("3")
    #print(move)
    pubname.publish(newECU)

def moving_callback_function(data):
    global still_moving, move
    if data.moving == True:
        still_moving = True
    else:
        move = False
        still_moving = False

# update
def callback_function(data):
    global move, still_moving
    # Convert the velocity into motorPWM and steering angle into servoPWM
    newECU.motor = ((0.7057*data.vel)/0.0121) + 1500
    #newECU.motor = ((0.8900*data.vel)/0.0121) + 1500
    newECU.servo = -(data.delta - 1.970449)/0.001309 

    maxspeed = 1580
    minspeed = 1400
    servomax = 1800
    servomin = 1200
    if (newECU.motor<minspeed):
        newECU.motor = minspeed
    elif (newECU.motor>maxspeed):
        newECU.motor = maxspeed
    if (newECU.servo<servomin):
        newECU.servo = servomin
    elif (newECU.servo>servomax):
        newECU.servo = servomax     # input steering angle

    if ((move == False) or (still_moving == False)):
        newECU.motor = 1500
        newECU.servo = 1540
    #print("5")
    #print(move)

    pubname.publish(newECU)

# state estimation node
def inputToPWM():
    
    # initialize node
    rospy.init_node('inputToPWM', anonymous=True)
    
    global pubname , newECU , subname, move , still_moving
    newECU = ECU() 
    newECU.motor = 1500
    newECU.servo = 1540
    move = False
    still_moving = False
    #print("1")
    #print(move)
    # topic subscriptions / publications
    pubname = rospy.Publisher('ecu_pwm',ECU, queue_size = 2)
    rospy.Subscriber('turtle1/cmd_vel', Twist, start_callback)
    subname = rospy.Subscriber('uOpt', Input, callback_function)
    rospy.Subscriber('moving', Moving, moving_callback_function)
    # set node rate
    loop_rate   = 40
    ts          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()
     
    rospy.spin()



if __name__ == '__main__':
    try:
       inputToPWM()
    except rospy.ROSInterruptException:
        pass
