import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys, select, os
from image_prediction import ImagePrediction
from lidar_prediction import LidarPrediction

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios


lidar = LidarPrediction()
image = ImagePrediction()


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.2


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


rospy.init_node('turtlebot3_teleop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

turtlebot3_model = rospy.get_param("model", "burger")



status = 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0

cls = {
    0: "Left",
    1: "Forward",
    2: "Right",
    3: "STOP!"
}

try:
    while True:
        ld_pred = lidar.get_prediction()
        #img_pred = image.get_prediction()

        if (ld_pred == 1): #Forward
            print('FORWARD --', ld_pred)
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
            status = status + 1

        elif ld_pred == 0: #Left
            print('LEFT --', ld_pred)
            target_angular_vel = checkAngularLimitVelocity(0 + ANG_VEL_STEP_SIZE)
            status = status + 1

        elif ld_pred == 2 : #Right
            print('RIGHT --', ld_pred)
            target_angular_vel = checkAngularLimitVelocity(0 - ANG_VEL_STEP_SIZE)
            status = status + 1

        if (ld_pred == 3): #Stop
            print('STOP! --', ld_pred)
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0


        twist = Twist()
        if (ld_pred == 1):
            #control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel = 0.1
        else:
            control_linear_vel = 0

        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)
        time.sleep(2)

except Exception as e:
    print("Issue: ", e)

finally:
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)
