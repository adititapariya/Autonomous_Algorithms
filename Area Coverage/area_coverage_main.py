#!/usr/bin/env python3
import rospy
import math
import time
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
from math import sin, cos, sqrt, tan, atan2, radians, degrees

rospy.init_node('areacoverage')

pub=rospy.Publisher('IMU_1', Quaternion ,queue_size=10)
pub1 = rospy.Publisher('/farm_bot/leftWheelfront_effort_controller/command', Float64, queue_size = 10)
pub2 = rospy.Publisher('/farm_bot/leftWheelrear_effort_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/farm_bot/rightWheelfront_effort_controller/command', Float64, queue_size = 10)
pub4 = rospy.Publisher('/farm_bot/rightWheelrear_effort_controller/command', Float64, queue_size = 10)

v1 = 0.06
v2= -0.06
i = 0
flag_var = 1
D = 2.0024e-07
dist = 0.0
tempd = [0,0]
tempfd = [0,0]
finalc = [0,0]
currentc = [0,0]
currentoc = [0,0]
temp1 = [0,0]
sub1 = None
sub2 = None
gps_msg = None
imu_msg = None
terminate1 = False
terminate2 = False
terminate3 = False
checkdist = float('inf')
j = 1
#distance D has 0.6531312064655274

def calculate_angle(yaw):
    t3 = +2.0 * (yaw.w * yaw.z + yaw.x * yaw.y)
    t4 = +1.0 - 2.0 * (yaw.y * yaw.y + yaw.z * yaw.z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def callbackIMU(msg):
    global imu_msg,theta
    imu_msg = msg
    # if i == 0:
    #     theta = calculate_angle(imu_msg)
    #     print("theta = ",theta)
    #     i = i + 1   
    print("aagaya")

def callbackGPS(msg):
    global gps_msg
    gps_msg = msg
    print("aagaya re")

def old2new(x,y):
    if theta > 0 : 
        x1 = -y*sin(abs(theta)) - x*cos(abs(theta))
        y1 = -y*cos(abs(theta)) + x*sin(abs(theta))
            
         
    else :
        x1= y*sin(abs(theta)) - x*cos(abs(theta))   
        y1= -y*cos(abs(theta)) - x*sin(abs(theta))
        
    return x1,y1
    
def new2old(x1,y1):
    if theta > 0:
        x = -x1*cos(abs(theta)) + y1*sin(abs(theta))
        y = -x1*sin(abs(theta)) - y1*cos(abs(theta))
        
    else :
        x = -x1*cos(abs(theta)) -  y1*sin(abs(theta))
        y = x1*sin(abs(theta)) - y1*cos(abs(theta))
        
    return x,y
    
def distance(tempd,gps_msg):            #distance with newplane coordinates
    R=6371.0        #radius of earth
    #print("tempd =", tempd)                     #distance of current and temporary destination using new plane coordinates
    sastac = storedata(gps_msg)
    currentc = old2new(sastac[0],sastac[1])
    dlon = tempd[1] - currentc[1]
    dlat = tempd[0] - currentc[0]
   
    global diffc             #array of dlat and dlon
    diffc = [dlat,dlon]
    print(diffc)
    a = sin(diffc[0] / 2)**2 + cos(currentc[0]) * cos(tempd[0]) * sin(diffc[1] / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    
    dist = Float64()
    dist = R*c*1000

    print("Result:", dist)          #to travel
    return dist

def storedata(gpsmsg):
    lat = round(radians(gps_msg.latitude),9)
    lon = round(radians(gps_msg.longitude),9)
    return lat,lon

def velo():
    
    global sub2a,terminate2,tempd, checkdist,dist,j
    print("checkdist = ",checkdist)
    if j > 1:
        checkdist = dist 
    if gps_msg is None:
        return
    sub2a = rospy.Subscriber('/fix', NavSatFix, callbackGPS, queue_size=1)   # Subscribe to GPS data
    rospy.sleep(0.1)
    sub2a.unregister()
    print("j=",j)
    print("checkdist_again = ",checkdist)
    dist = distance(tempd,gps_msg)  

    print("Result:", dist)  # to travel
    j = j + 1
    
    if checkdist < dist :
        print("stop")
        pub1.publish(0.0)
        pub2.publish(0.0)
        pub3.publish(0.0)
        pub4.publish(0.0)
        rospy.sleep(0.1)
        terminate2 = True
        

    if dist >= 2:
        pub1.publish(0.0256)
        pub2.publish(0.0256)
        pub3.publish(0.0256)
        pub4.publish(0.0256)

    elif dist < 2:
        if dist >= 0.3:
            pub1.publish(0.0175 + (0.004 * dist))
            pub2.publish(0.0175 + (0.004 * dist))
            pub3.publish(0.0175 + (0.004 * dist))
            pub4.publish(0.0175 + (0.004 * dist))
           
        else :
            pub1.publish(0.0)
            pub2.publish(0.0)
            pub3.publish(0.0)
            pub4.publish(0.0)
            print("aaila")
            terminate2 = True

def angletogoal(temp1):
    global sub1, sub2
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
    #dist = Float64()
    #dist = R*c*1000
    #print("Result:", dist)          #to travel
    currentoc = storedata(gps_msg)
    dlona = temp1[1] - currentoc[1]    #using both coordinates in oldplane system for orientation
    dlata = temp1[0] - currentoc[0]
   
    global diffca            #array of dlat and dlon
    diffca = [dlata,dlona]
    print(diffca)
    ag = sin(diffca[0] / 2)**2 + cos(currentoc[0]) * cos(temp1[0]) * sin(diffca[1] / 2)**2
    cg = 2 * atan2(sqrt(ag), sqrt(1 - ag))
    global A
    A = Float64()
    A = round(atan2 (diffca[0], diffca[1]),9)
    print('angle_to_goal: ',A)
    YAW = calculate_angle(imu_msg.orientation)
    global angle,terminate1,terminate2
    angle = Float64()
    angle = A - YAW
    print("angle =",angle)
    print(" ") 
   
    if angle > 0.08 and angle < 3.132:
       
        pub1.publish(v2)
        pub2.publish(v2)
        pub3.publish(v1)
        pub4.publish(v1)
        
    elif angle > 3.132 and angle < 6.272:
        pub1.publish(v1)
        pub2.publish(v1)
        pub3.publish(v2)
        pub4.publish(v2)
    
    elif angle < -0.08 and angle > -3.132:
        pub1.publish(v1)
        pub2.publish(v1)
        pub3.publish(v2)
        pub4.publish(v2)
        
    elif angle > -3.132 and angle < -6.272:
        pub1.publish(v2)
        pub2.publish(v2)
        pub3.publish(v1)
        pub4.publish(v1)

    else :
        terminate1 = True
       
#tempfd = temporary destination coordinates in new plane
#tempd = temporary destination coordinates in new plane*
#temp1 = temporary destination coordinates in old plane for orientation

def movement():
    global flag_var,temp1,checkdist,j 
    rate = rospy.Rate(1)  # Adjust the rate as needed
    while not rospy.is_shutdown() and not terminate1:
        print("entered")
        angletogoal(temp1)
        rate.sleep()
    while not rospy.is_shutdown() and not terminate2:
        print("entered 2")
        velo()
        rate.sleep()
    flag_var = flag_var + 1
    j = 1
    checkdist = float('inf')

def check_flag(flag_var,h,j,k):       #h is the larger coordinate where the motion starts initially
    global tempfd,tempd,temp1,terminate1,terminate2
    if flag_var == 1:
        print("1st manoeuvre")
        print("h =", h)
        print("tempfd =", tempfd)
        tempfd = [xf,0]
        tempd = tempfd
        temp1 = new2old(tempfd[0],tempfd[1])
        print("tempd =", tempd)
        movement() 

    elif flag_var%4 == 1:
        print("1st manoeuvre")
        tempfd[j]=tempfd[j]+h
        tempd = tempfd
        temp1 = new2old(tempfd[0],tempfd[1])
        print("tempd =", tempd)
        movement()     
         
    elif flag_var%4 ==  2:
        print("2nd manoeuvre")
        tempfd[k]=tempfd[k]+D
        tempd = tempfd
        temp1 = new2old(tempfd[0],tempfd[1])
        terminate1 = False
        terminate2 = False
        print("tempd =", tempd)
        movement()      
        
    elif flag_var%4 == 3:
        print("3rd manoeuvre")
        tempfd[j]=tempfd[j]-h
        tempd = tempfd
        temp1 = new2old(tempfd[0],tempfd[1])
        terminate1 = False
        terminate2 = False
        print("tempd =", tempd)
        movement()
           
    else :
        print("4th manoeuvre")
        tempfd[k]=tempfd[k]+D
        tempd = tempfd
        temp1 = new2old(tempfd[0],tempfd[1])
        terminate1 = False
        terminate2 = False
        print("tempd =", tempd)
        movement()
        
def main():
    global latf,lonf,sub1,sub2,xf,yf,R,finalc,Distf,theta,D,terminate3
    Distf = Float64()
    latf = radians(2.7833e-05)    #specified destination
    lonf = radians(3.4668e-05)
    if sub1 is not None:
        sub1.unregister()
    if sub2 is not None:
        sub2.unregister()
        
    sub2 = rospy.Subscriber('/fix', NavSatFix, callbackGPS, queue_size=1)   # Subscribe to GPS data
    print("check1")
    while not rospy.is_shutdown() and gps_msg is None:              # Wait until GPS data is available
        rospy.sleep(0.01)
        print("check2")
    sub2.unregister()                   #Unsubscribe from GPS data              
    
    sub1 = rospy.Subscriber('/imu', Imu, callbackIMU, queue_size=1)     # Subscribe to IMU data

    while not rospy.is_shutdown() and imu_msg is None:                  # Wait until IMU data is available
        rospy.sleep(0.01)
        print("check3")
    sub1.unregister()          # Unsubscribe from IMU data
    
    theta = calculate_angle(imu_msg.orientation)
    print("theta = ",theta)
    if theta > 0: 
        xf= -lonf*sin(abs(theta)) - latf*cos(abs(theta))   
        yf= -lonf*cos(abs(theta)) + latf*sin(abs(theta))  #destination coordinates in new plane
       
    else :
       
        xf= lonf*sin(abs(theta)) - latf*cos(abs(theta))   
        yf= -lonf*cos(abs(theta)) - latf*sin(abs(theta))
                                                         
    print("xf = ", xf)                     
    print("yf =", yf)
    print(" ") 
    finalc = [xf,yf]
    R=6371.0        #radius of earth
    
    Distf = distance(finalc,gps_msg)
    print("Final_distance",Distf)
    
    if Distf > 0.6:
        if abs(yf)>=abs(xf):
            if xf < 0:
                D = -D
            print("entered yf")
            while not rospy.is_shutdown() and not terminate3:
                check_flag(flag_var,yf,1,0)
                Distf= distance(finalc,gps_msg)
                print("Distf = ",Distf)
                if Distf < 0.6 :
                    terminate3 = True
            print("khatam bc")
      
        else :
            if yf < 0:
                D = -D    
            print("entered xf")
            while not rospy.is_shutdown() and not terminate3:
                check_flag(flag_var,xf,0,1)
                Distf= distance(finalc,gps_msg)
                print("Distf = ",Distf)
                if Distf < 0.6 :
                    terminate3 = True
            print("khatam bc")
    else :
        pub1.publish(0.00)
        pub2.publish(0.00)
        pub3.publish(0.00)
        pub4.publish(0.00)
        
if __name__ == '__main__':
    main()
