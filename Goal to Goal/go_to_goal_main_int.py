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

pub = rospy.Publisher('IMU_1', Quaternion, queue_size=10)
pub1 = rospy.Publisher('/farm_bot/leftWheelfront_effort_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/farm_bot/leftWheelrear_effort_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/farm_bot/rightWheelfront_effort_controller/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/farm_bot/rightWheelrear_effort_controller/command', Float64, queue_size=10)

v1 = 0.06
v2 = -0.06
flag = 0
dlon = 0
dlat = 0
theta = 0
terminate1 = False
terminate2 = False
sub1 = None
sub2 = None
gps_msg = None
imu_msg = None

def callbackIMU(msg):
    global imu_msg
    imu_msg = msg
    print("aagaya")

def callbackGPS(msg):
    global gps_msg
    gps_msg = msg
    print("aagaya re")

def distance(gps_msg):
    global R
    R=6371.0        #radius of earth
    # print(gps_msg.latitude)
    # print(gps_msg.longitude)
    lat1 = radians(gps_msg.latitude)
    lon1 = radians(gps_msg.longitude)
    lat2 = radians(goal_lat)
    lon2 = radians(goal_lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    dist = R * c * 1000  # distance in meters
    return dist

def calculate_angle(yaw):
    t3 = +2.0 * (yaw.w * yaw.z + yaw.x * yaw.y)
    t4 = +1.0 - 2.0 * (yaw.y * yaw.y + yaw.z * yaw.z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def dgps(gps_msg):
    lat1 = radians(gps_msg.latitude)
    lon1 = radians(gps_msg.longitude)
    lat2 = radians(goal_lat)
    lon2 = radians(goal_lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1
    return atan2(dlat,dlon)

def velo():
    global dlat, dlon,sub2a
    if gps_msg is None:
        return
    sub2a = rospy.Subscriber('/fix', NavSatFix, callbackGPS, queue_size=1)   # Subscribe to GPS data
    rospy.sleep(0.03)
    sub2a.unregister()
    dist = distance(gps_msg)

    print("Result:", dist)  # to travel

    if dist >= 2:
        pub1.publish(0.025)
        pub2.publish(0.025)
        pub3.publish(0.025)
        pub4.publish(0.025)

    elif dist < 2:
        if dist >= 0.6:
            pub1.publish(0.017 + (0.003 * dist))
            pub2.publish(0.017 + (0.003 * dist))
            pub3.publish(0.017 + (0.003 * dist))
            pub4.publish(0.017 + (0.003 * dist))
           

        else :
            pub1.publish(0.0)
            pub2.publish(0.0)
            pub3.publish(0.0)
            pub4.publish(0.0)
            print("aaila")
            terminate1 = True

def angletogoal():
    global A,sub1,sub2,theta, dlat, dlon, gps_msg, imu_msg,goal_lat,goal_lon
    goal_lat = 4.602860810234308e-05
    goal_lon = 5.44751881762401e-09
    print("idhar bhi aaya")
     # Unsubscribe from GPS and IMU data
    if sub1 is not None:
        sub1.unregister()
    if sub2 is not None:
        sub2.unregister()
        
    sub2 = rospy.Subscriber('/fix', NavSatFix, callbackGPS, queue_size=1)   # Subscribe to GPS data
    print("check1")
    while not rospy.is_shutdown() and gps_msg is None:              # Wait until GPS data is available
        rospy.sleep(0.001)
        print("check2")
    sub2.unregister()                   #Unsubscribe from GPS data              
    
    sub1 = rospy.Subscriber('/imu', Imu, callbackIMU, queue_size=1)     # Subscribe to IMU data

    while not rospy.is_shutdown() and imu_msg is None:                  # Wait until IMU data is available
        rospy.sleep(0.001)
        print("check3")
    sub1.unregister()          # Unsubscribe from IMU data
    

    # Subscribe to GPS and IMU data again
    sub2 = rospy.Subscriber('/fix', NavSatFix, callbackGPS, queue_size=1)
    print("present")
    sub1 = rospy.Subscriber('/imu', Imu, callbackIMU, queue_size=1)

    # Wait for 0.25 seconds to receive new GPS and IMU data
    rospy.sleep(0.25)

    # Unsubscribe from GPS and IMU data again
    sub2.unregister()
    sub1.unregister()

    # Calculate angle based on IMU data
    theta = calculate_angle(imu_msg.orientation)
    print("theta =", theta)
    A = Float64()
    A = dgps(gps_msg)
    print('angle_to_goal: ',A)
    global angle, terminate1,terminate2
    angle = Float64()
    angle = A - theta
    print("angle =",angle)
    dist = distance(gps_msg)
    print("Result:", dist)  # to travel
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
        terminate1 = True

def main():
    rate = rospy.Rate(1)  # Adjust the rate as needed
    while not rospy.is_shutdown() and not terminate1:
        print("entered")
        angletogoal()
        rate.sleep()
    while not rospy.is_shutdown() and not terminate2:
        print("entered 2")
        velo()
        rate.sleep()
    print("khatam bc")
if __name__ == '__main__':
    main()