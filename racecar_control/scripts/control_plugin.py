#!/usr/bin/env python
#coding=utf-8
import rospy
import math
import sys
import tf2_ros
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

'''
MAIN FUNCTION
订阅twist_ackermann话题将指令发到控制器上
'''
wheelbase = 0.335
wheeltrack = 0.276
tire_dia = 0.14605    #轮胎直径
# vehicle name

car_name = str(sys.argv[1])

# subscriber topics
command_topic     = '/{}/command'.format(car_name)
pub_vel_LRW = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
pub_vel_RRW = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
pub_vel_LFW = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
pub_vel_RFW = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
pub_pos_LSH = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
pub_pos_RSH = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)


def in_out(sita_in):
    sita_out = math.atan(1/(1/math.tan(abs(sita_in))+wheeltrack/wheelbase))
    return sita_out

#回调函数
def command_callback(data):
    if data.speed >= 0:
        w = abs(data.steering_angle_velocity)
    if data.speed < 0:
        w = -abs(data.steering_angle_velocity)

    steering_angle_l = Float64()
    steering_angle_r = Float64()

    speed_lr          = Float64()   #左后轮
    speed_rr          = Float64()   #右后轮
    speed_lf          = Float64()   #左前轮
    speed_rf          = Float64()   #右前轮

    #根据传递来的角度进行分类
    if  data.steering_angle > 0 :
        radius = data.speed / data.steering_angle_velocity
        steering_angle_l = data.steering_angle
        steering_angle_r = in_out(data.steering_angle)
        speed_lr = w * (radius - wheeltrack/2) * 2/ tire_dia
        speed_rr = w * (radius + wheeltrack/2) * 2/ tire_dia
        speed_lf = w * wheelbase / math.sin(steering_angle_l) * 2/ tire_dia
        speed_rf = w * wheelbase / math.sin(steering_angle_r) * 2/ tire_dia

    if  data.steering_angle < 0 :
        radius = abs(data.speed / data.steering_angle_velocity)
        steering_angle_r = data.steering_angle
        steering_angle_l = -in_out(data.steering_angle)
        speed_lr = w * (radius + wheeltrack/2) * 2/ tire_dia
        speed_rr = w * (radius - wheeltrack/2) * 2/ tire_dia
        speed_lf = w * wheelbase / math.sin(abs(steering_angle_l)) * 2/ tire_dia
        speed_rf = w * wheelbase / math.sin(abs(steering_angle_r)) * 2/ tire_dia
   
    if  data.steering_angle == 0 :
        steering_angle_l = 0
        steering_angle_r = 0
        speed_lr = data.speed * 2/ tire_dia
        speed_rr = data.speed * 2/ tire_dia
        speed_lf = data.speed * 2/ tire_dia
        speed_rf = data.speed * 2/ tire_dia

    #发布话题
    pub_vel_LRW.publish(speed_lr)
    pub_vel_RRW.publish(speed_rr)
    pub_vel_LFW.publish(speed_lf)
    pub_vel_RFW.publish(speed_rf)

    pub_pos_LSH.publish(steering_angle_l)
    pub_pos_RSH.publish(steering_angle_r)


if __name__ == '__main__':

    try:

        rospy.init_node('control_plugin', anonymous = True)
        rospy.Subscriber(command_topic, AckermannDrive, command_callback)

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
