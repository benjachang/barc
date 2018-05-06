#!/usr/bin/env python

import rospy
from barc.msg import ECU, Encoder
from pid import PID
from observer import EncoderModel, ImuModel
from sensor_msgs.msg import Imu
from numpy.random import uniform
from numpy import around

def main():

    # initialize velocity estimator
    enc = EncoderModel()
    imu = ImuModel()

    # initialize node, set topic subscriptions / publications
    rospy.init_node('driftController', anonymous=True)
    rospy.Subscriber('/encoder', Encoder, enc.estimateVelocityM1)
    rospy.Subscriber('/imu/data', Imu, imu.updateEstimates)
    ecu_pub   = rospy.Publisher('/ecu_pwm', ECU, queue_size = 10)

    # initialize pid controller
    loop_rate   = rospy.get_param("controller/loop_rate")
    v_ref       = rospy.get_param("controller/v_ref")
    Pm          = rospy.get_param("controller/Pm")
    Im          = rospy.get_param("controller/Im")
    Dm          = rospy.get_param("controller/Dm")
    Ps          = rospy.get_param("controller/Ps")
    Is          = rospy.get_param("controller/Is")
    Ds          = rospy.get_param("controller/Ds")

    # pid controller
    rate        = rospy.Rate(loop_rate)
    dt          = 1.0/loop_rate
    pid_motor   = PID(Pm,Im,Dm)
    pid_servo   = PID(Ps,Is,Ds)
    pid_motor.setTimeStep(dt)
    pid_servo.setTimeStep(dt)
    pid_motor.setPoint(v_ref)
    pid_servo.setPoint(0.0)

    # sample input parameters for drift
    s               = 2.0   # distance to go straight
    Dt_acc          = 0.35  # time to turn/accelerate #uniform(0,0.8)
    Dt_brk          = 0.25  # time to brake
    df_right        = 1850  # right turn steering angle #int( uniform( 1850, 1900 ) )
    df_left         = 1150  # left turn steering angle
    acc_PWM         = 1873  # accelearate PWM #int( uniform( 1850, 1900 ) )
    brk_PWM         = 990   # brake PWM
    totalNumDrifts  = 2     # total number of drift maneuvers

    u_motor_neutral = 1500
    u_servo_neutral = 1500
    u_motor         = u_motor_neutral
    u_servo         = u_servo_neutral

    rospy.logwarn("Dt = {}".format(Dt))
    rospy.logwarn("F1 (acceleration) = {}".format(F1))
    rospy.logwarn("df (steering) = {}".format(df))

    now         = rospy.get_rostime()
    t0          = now.secs + now.nsecs/(10.0**9)
    straight    = False        
    turn        = False
    brake       = False

    numDrifts   = 0
 
    while not rospy.is_shutdown():
        # get time
        # get time
        now = rospy.get_rostime()
        t   = now.secs + now.nsecs/(10.0**9) - t0

        if numDrifts < totalNumDrifts:

            # get vehicle into initial state
            # if enc.s_m1 < s: BUG: enc.s_m1 seems to grow more negative
    	    if enc.s_m1 > -s:
                # rospy.logwarn("s1 = {}".format(enc.s_m1))
                if not straight:
                    rospy.logwarn("Going straight ...")
                    straight = True

                 # compute feedforward / feedback command for motor
                u_ff    = u_motor_neutral
                u_fb    = pid_motor.update( enc.vhat_m1 )
                u_motor = u_ff + int(u_fb)
                
                # compute feedforward / feedback command for servo
                u_ff    = u_servo_neutral
                u_fb    = pid_servo.update( -imu.dy_deg )
                u_servo = u_ff + int(u_fb)

                t_straight  = t
            
            # perform aggresive turn and accelerate
            elif t < t_straight + Dt_acc:
                if not turn:
                    rospy.logwarn("Turning and accelerating ...")
                    turn = True
                u_motor = acc_PWM 
                u_servo = df_right

            # apply brake
            elif t < t_straight + Dt_acc + Dt_brk:
                if not brake:   
                    rospy.logwarn("Braking ! ...")
                    brake = True
                u_motor = brk_PWM
                u_servo = u_servo_neutral
            
            # reset for next drift sequence
            elif t > t_straight + Dt_acc + Dt_brk:
                rospy.logwarn("Resetting for next drift ...")
                enc.s_m1 = 0
                numDrifts = numDrifts + 1
                straight    = False        
                turn        = False
                brake       = False

        # publish control command
        #rospy.logwarn("v1 = {}".format(enc.vhat_m1))
        rospy.logwarn("s1 = {}".format(enc.s_m1))
        #rospy.logwarn("yaw = {}".format(imu.dy))
        #rospy.logwarn("numDrifts = {}".format(numDrifts))
        #print('numDrifts: ', numDrifts)
        ecu_pub.publish( ECU(u_motor, u_servo) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       main()
        now = rospy.get_rostime()
        t   = now.secs + now.nsecs/(10.0**9) - t0
    except rospy.ROSInterruptException:
        pass
