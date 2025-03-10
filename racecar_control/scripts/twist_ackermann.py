#!/usr/bin/env python
# coding=utf-8
import rospy
import sys
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

'''
MAIN FUNCTION
接受L1_controller_v2 发出的速度和打角指令 /car/cmd_vel
'''
car_name = str(sys.argv[1])
wheelbase = 0.335  # 轴距
wheeltrack = 0.276  # 轮距

command_pub = rospy.Publisher(
    '/{}/command'.format(car_name), AckermannDrive, queue_size=1)

# ackermann运动模型公式转换


def twist_ackermann_node(data):
    command = AckermannDrive()
    command.speed = data.linear.x
    if data.angular.z > 0:
        radius = (data.linear.x/data.angular.z) * (180.0 / 3.14159265358979)
        command.steering_angle = math.atan(wheelbase/(radius-wheeltrack/2))
    if data.angular.z < 0:
        radius = (data.linear.x/data.angular.z) * (180.0 / 3.14159265358979)
        command.steering_angle = - \
            math.atan(wheelbase/(abs(radius)-wheeltrack/2))
    command.steering_angle_velocity = data.angular.z*(3.14159265358979 / 180.0)


    command_pub.publish(command)
    '''
    baseSpeed = 0
    baseAngle = 0
    command                = AckermannDrive()
    if data.linear.x == 1500 or data.angular.z == 90:
        command.steering_angle = 0.0
        command.speed = 0.0
    else:
        if data.angular.z > 90:
            radius = (baseSpeed-data.linear.x)/(data.angular.z-baseAngle)
            command.steering_angle = math.atan(wheelbase/(radius-wheeltrack/2))
        if data.angular.z < 90:
            radius = (baseSpeed-data.linear.x)/(data.angular.z-baseAngle)
            command.steering_angle = -math.atan(wheelbase/(abs(radius)-wheeltrack/2))
    command.speed = baseSpeed-data.linear.x
    command.steering_angle_velocity = data.angular.z
    command_pub.publish(command)
    '''
    '''
    if data.linear.x == 0.0 or data.angular.z == 0.0:
        command.steering_angle = 0.0
    else:
        if data.angular.z > 0:
            radius = data.linear.x/data.angular.z
            command.steering_angle = math.atan(wheelbase/(radius-wheeltrack/2))
        if data.angular.z < 0:
            radius = data.linear.x/data.angular.z
            command.steering_angle = - math.atan(wheelbase/(abs(radius)-wheeltrack/2))
    command.speed = data.linear.x
    command.steering_angle_velocity = data.angular.z
    command_pub.publish(command)
    '''


if __name__ == '__main__':
    try:
        rospy.init_node('twist_ackermann_node', anonymous=True)
        rospy.Subscriber('/car/cmd_vel', Twist, twist_ackermann_node)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
