import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys, select, os
from camera_capture import GetImageAsArray
from laser_scan import Lidar
import tflite_runtime.interpreter as tflite


picam = GetImageAsArray()
lidar = Lidar(150)

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

def classify_movement():
    global picam, lidar

    image = (picam.get_image_array()).flatten()
    distances = normalize(lidar.get_distance())
    data = np.concatenate((distances, image))

    interpreter = tflite.Interpreter(model_path="self_driving_5k_v2.tflite")
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    input_shape = input_details[0]['shape']
    input_instance = np.expand_dims(data, axis=0).astype(np.float32)

    interpreter.set_tensor(input_details[0]['index'], input_instance)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_details[0]['index'])
    output_data = (list(map(np.float32, output_data[0])))
    result = output_data.index(max(output_data))

    return result

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


rospy.init_node('turtlebot3_teleop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

turtlebot3_model = rospy.get_param("model", "burger")


status = 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0


cls_labels = {
    1: 'w',
    0: 'a',
    2: 'd',
    3: 's'
}

lbls = {
    1: 'Forward',
    0: 'Left',
    2: 'Right',
    3: 'Stop'
}

print("Begin..")

while True:
    print(lbls[classify_movement()])
    time.sleep(0.5)

key = 's'
try:
    while True:

        if (key == 'w'): #Forward
            print('FORWARD')
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
            status = status + 1

        elif (key == 'a'): #Left
            print('LEFT')
            target_angular_vel = checkAngularLimitVelocity(0 + ANG_VEL_STEP_SIZE)
            status = status + 1

        elif (key == 'd'): #Right
            print('RIGHT')
            target_angular_vel = checkAngularLimitVelocity(0 - ANG_VEL_STEP_SIZE)
            status = status + 1

        elif (key == 's'): #Stop
            print('STOP!')
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0

        elif (getKey() == 'q'):
            break


        twist = Twist()
        if (key == 'w'):
            #control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel = 0.1
        else:
            control_linear_vel = 0

        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)
        time.sleep(0.5)


        key = cls_labels[classify_movement()]


except Exception as e:
    print("Issue: ", e)

finally:
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
