import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class Lidar():
    def __init__(self, degrees):
        self.init_ros()
        self.DEGREES = degrees

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan, 10)

        samples = len(scan.ranges)

        lst = list(scan.ranges[-(self.DEGREES//2):])
        lst.extend(scan.ranges[:(self.DEGREES//2)])
        lst = lst[::-1]
        lst = np.array(lst)

        for i in range(len(lst)):
            if lst[i] == float('Inf'):
                lst[i] = 3.5
            elif math.isnan(lst[i]):
                lst[i] = 0

        return lst

    def get_distance(self):
        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            return np.array(lidar_distances)

    def init_ros(self):
        pass
        #rospy.init_node('turtlebot3_obstacle', disable_signals=True)
