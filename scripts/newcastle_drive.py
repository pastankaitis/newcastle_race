#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

import os
import numpy as np
import tf
import math
import sys

# PID CONTROL PARAMS
kp = 1.115
kd = 0.08
ki = 0.0000015


L = 0.4



servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0


class NewcastleDrive(object):

    def __init__(self):

        self.index = 0

        self.scan_sub = rospy.Subscriber('/tyne_team/scan', LaserScan, self.scan_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('/tyne_team/odom', Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('/tyne_team/drive', AckermannDriveStamped, queue_size=1)


    def select_velocity(self, angle):
        if abs(angle) <= 5 * math.pi / 180:
            velocity = 7.0
        elif abs(angle) <= 10 * math.pi / 180:
            velocity = 6.5
        elif abs(angle) <= 15 * math.pi / 180:
            velocity = 6.5
        elif abs(angle) <= 20 * math.pi / 180:
            velocity = 6.0
        else:
            velocity = 5.0
        return velocity

    def findangle(self, data):
        lid = []
        maxindex = 540
        i = 0
        x = 0
        readingold = 0
        gs = 0
        lgs = 0
        reading = 0
        z = 0
        while z < len(data.ranges):
            if data.ranges[z] >= 3 and (z > 180) and (z < 900):
                gs += 1
                if gs > lgs:
                    lgs = gs
            else:
                gs = 0

            z += 1

        while i < len(data.ranges):
            if (i <= 300) or (i >= 780):
                x = 0
                reading = 0

            elif data.ranges[i] <= 3.5 and lgs > 80:
                x = 0
                reading = 0

            elif data.ranges[i] <= 1.6:
                x = 0
                reading = 0

            else:
                reading += data.ranges[i] - 0.005 * abs(540 - i)
                x += 1
                if x > 10 and reading / x ** 0.3 > readingold:
                    readingold = reading / x ** 0.3
                    maxindex = i - x / 2

                if lgs < 130 and maxindex > 540:
                    maxindex += 40
                if lgs < 130 and maxindex < 540:
                    maxindex += -40

                print(lgs, " ", maxindex, " ", x)
            i += 1
        # print(len(lid))
        return maxindex

    def driver(self, angle, velocity):

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, data):

        self.lidar = data

        # publish drive message

    def pose_callback(self, pose_msg):

        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                               pose_msg.pose.pose.orientation.y,
                               pose_msg.pose.pose.orientation.z,
                               pose_msg.pose.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]

       

        anglefound = self.findangle(self.lidar)
        angle_to_dist = (135 - anglefound / 4)

        # prev_angle = 0

        # if (angle_to_dist)

        global integral
        global prev_error
        global kp
        global ki
        global kd

        

        pid_angle = -kp * angle_to_dist  

        
        if (angle_to_dist > 40) or (angle_to_dist < -40):
            pid_angle = np.clip(pid_angle, -0.4, 0.4)
        else:
            pid_angle /= 100

        # get lidar data with self.data from scan_callback
        self.driver(pid_angle, self.select_velocity(pid_angle))


if __name__ == '__main__':
    rospy.init_node('newcastle_drive_node')
    nd = NewcastleDrive()
    rospy.spin()
