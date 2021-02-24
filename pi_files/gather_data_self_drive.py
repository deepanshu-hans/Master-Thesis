import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys, select, os
from camera_capture import GetImageAsArray
from laser_scan import Lidar
import joblib
import pandas as pd
from tqdm import tqdm
import csv
from lidar_prediction import LidarPrediction

picam = GetImageAsArray()
lidar = Lidar(150)
lidar_pred = LidarPrediction()

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
    settings = termios.tcgetattr(sys.stdin)



BURGER_MAX_LIN_VEL = 0.15
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.3


def normalize(distances):
    distances = [val if val > 0.01 else 1.0 for val in distances]
    distances = [val if val < 1.0 else 1.0 for val in distances]
    return np.array(distances, dtype = np.float32)


def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


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


def write_data_to_disk(data, pub):
    stop_bot(pub)
    time.sleep(1)
    print("Saving...", len(data))
    with open('saved.csv', 'a', newline='') as out_file:
        writer = csv.writer(out_file)
        for i in tqdm(range(len(data))):
            writer.writerow(data[i])
    out_file.close()
    print("Data Saved!")


def stop_bot(pub):
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)


rospy.init_node('turtlebot3_teleop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

turtlebot3_model = rospy.get_param("model", "burger")

status = 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0


cls_labels = {
    1: 'Forward',
    0: 'Left',
    2: 'Right',
    3: 'Stop!'
}

print("Begin..")
data = []

key = 3
try:
    while True:
        #curr_key = getKey()
        #if curr_key == '':
        #    key = prev_key
        #else:
        #    key = curr_key

        if (key == 1): #Forward
            stop_bot(pub)
            time.sleep(0.2)
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
            status = status + 1

        elif (key == 0): #Left
            target_angular_vel = checkAngularLimitVelocity(0 + ANG_VEL_STEP_SIZE)
            status = status + 1

        elif (key == 2): #Right
            target_angular_vel = checkAngularLimitVelocity(0 - ANG_VEL_STEP_SIZE)
            status = status + 1

        elif (key == 3): #Stop
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0

        if (len(data) >= 250):
            write_data_to_disk(data, pub)
            break

        print(cls_labels[key], len(data))

        twist = Twist()
        if (key == 1):
            #control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel = 0.1
            target_angular_vel  = 0.0
            control_angular_vel = 0.0
        else:
            control_linear_vel = 0

        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)

        key, distances = lidar_pred.get_prediction()
        image = (picam.get_image_array()).flatten()
        instance = np.concatenate((distances, image, np.array([key])))
        data.append(instance.astype(np.float32))
        time.sleep(0.2)


except Exception as e:
    print("Issue: ", e)
    stop_bot(pub)
    print("Bot Stopped!")
    write_data_to_disk(data, pub)

finally:
    stop_bot(pub)
    print("Bot Stopped!")

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
