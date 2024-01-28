#!/usr/bin/env python
import rospy
import math
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
from math import sin, cos, sqrt, atan2, radians
rospy.init_node('goal_to_goal')

pub=rospy.Publisher('IMU_1', Quaternion ,queue_size=10)
pub1 = rospy.Publisher('/farm_bot/leftWheelfront_effort_controller/command', Float64, queue_size = 10)
pub2 = rospy.Publisher('/farm_bot/leftWheelrear_effort_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/farm_bot/rightWheelfront_effort_controller/command', Float64, queue_size = 10)
pub4 = rospy.Publisher('/farm_bot/rightWheelrear_effort_controller/command', Float64, queue_size = 10)
v1 = 0.06
v2 = -0.06

def velo():
    
    if dist >= 2 :
        
        pub1.publish(0.05)
        pub2.publish(0.05)
        pub3.publish(0.05)
        pub4.publish(0.05)
    
    elif dist < 2 :
        
        pub1.publish(0.008 * dist)
        pub2.publish(0.008 * dist)
        pub3.publish(0.008 * dist)
        pub4.publish(0.008 * dist)
        
  
def callback(msg):

    global x
    x = Float64()
    x = msg.orientation.x
    #print('x=', x)
   
    global y
    y = Float64()
    y = msg.orientation.y
    #print('y=', y)
    
    global z
    z = Float64()
    z = msg.orientation.z
    #print('z=', z)
    
    global w
    w = Float64()
    w = msg.orientation.w
    #print('w=', w)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    global theta
    theta = yaw_z
    
    print('YAW=',yaw_z)

def callback1(msg):
    global Lat
    Lat = Float64()
    Lat = msg.latitude
    print ("Latitude= ", Lat)
   
    global Long
    Long = Float64()
    Long = msg.longitude
    print ("Longitude= ", Long)
    #print(" ")

    R=6371.0        #radius of earth

    lat1 = radians(Lat)
    lon1 = radians(Long)
    lat2 = radians(-4.4679e-05)  #specified destination
    lon2 = radians(-4.6013e-05)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    
    global dist  
    dist = Float64()
    dist = R*c*1000

    print("Result:", dist)          #to travel
   
    global A
    A = Float64()
    A = atan2 (dlat, dlon)
    print('angle_to_goal: ',A)
    global angle 
    angle = Float64()
    angle = A - theta
    print("angle =",angle)
    print(" ")
    
    if angle > 0.1 and angle < 3.13:
        pub1.publish(v2)
        pub2.publish(v2)
        pub3.publish(v1)
        pub4.publish(v1)
        
    elif angle > 3.13 and angle < 6.27:
        pub1.publish(v1)
        pub2.publish(v1)
        pub3.publish(v2)
        pub4.publish(v2)
    
    elif angle < -0.1 and angle > -3.13:
        pub1.publish(v1)
        pub2.publish(v1)
        pub3.publish(v2)
        pub4.publish(v2)
        
    elif angle > -3.13 and angle < -6.27:
        pub1.publish(v2)
        pub2.publish(v2)
        pub3.publish(v1)
        pub4.publish(v1)
        
    else :
        velo()

sub = rospy.Subscriber('/imu', Imu , callback)
sub = rospy.Subscriber('/fix', NavSatFix, callback1)
rospy.spin()
