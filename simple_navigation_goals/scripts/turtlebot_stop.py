#!/usr/bin/env python3
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
#from geometry_msgs.msg import Twist


#LINEAR_VEL = 0.22
STOP_DISTANCE = 1.00
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
def clear_costmap():

    rospy.wait_for_service('/move_base/clear_costmaps')
    print("fdf")
    try:

        clear = rospy.ServiceProxy('/move_base/clear_costmaps',)
        clear()
        print('Costmap cleared')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return


class Obstacle():
    def __init__(self):
        #self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self._pub = rospy.Publisher('stop',Bool,queue_size=10)
        self.rate =rospy.Rate(100)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        print(scan_filter)
        
        return scan_filter

    def obstacle(self):
        #twist = Twist()
        turtlebot_moving = True
        #self._pub.publish(False)
                    


        #rate =rospy.Rate(0.05)

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
            
            rospy.loginfo('Distance of the obstacle : %f', min_distance)

            if min_distance < SAFE_STOP_DISTANCE:
            	if turtlebot_moving:
                    #self._pub.publish(True)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
                    rospy.set_param('/stop_robot', True)

            else :
                turtlebot_moving=True
                rospy.loginfo('Path is free')
                rospy.set_param('/stop_robot',False)


def main():
    rospy.init_node('turtlebot3_obstacle')
    print("obstacle stopping node started")
    rospy.set_param('/stop_robot', False)
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
