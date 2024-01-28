#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
import time

pub1 = rospy.Publisher('/farm_bot/leftWheelfront_effort_controller/command', Float64, queue_size = 10)
pub2 = rospy.Publisher('/farm_bot/leftWheelrear_effort_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/farm_bot/rightWheelfront_effort_controller/command', Float64, queue_size = 10)
pub4 = rospy.Publisher('/farm_bot/rightWheelrear_effort_controller/command', Float64, queue_size = 10)

rospy.init_node('farmbot')

def callback(msg):
    global F
    F = Float32()
    F = msg.range
    print(F)
    #print('F=', F.data)
F = rospy.Subscriber('sensor/ir_front', Range, callback)
def callback(msg):
    global L
    L = Float32()
    L = msg.range
    print(L)
    #print('L=', L.data)
L = rospy.Subscriber('sensor/ir_left', Range, callback)
def callback(msg):
    global LS
    LS = Float32()
    LS = msg.range
    print(LS)
    #print('LS=', LS.data)
LS = rospy.Subscriber('sensor/ir_leftslant', Range, callback)
def callback(msg):
    global R
    R = Float32()
    R = msg.range
    print(R)
    #print('R=', R.data)
R = rospy.Subscriber('sensor/ir_right', Range, callback)
def callback(msg):
    global RS
    RS = Float32()
    RS = msg.range
    print(RS)
    #print('RS=', RS.data)
RS = rospy.Subscriber('sensor/ir_rightslant', Range, callback)
vf = 0.03
vb = -0.03
rate = rospy.Rate(10)
while not rospy.is_shutdown():
   if F == 1.0 and LS == 0.75 and RS == 0.75 :
     pub1.publish(vf)
     pub2.publish(vf)
     pub3.publish(vf)
     pub4.publish(vf) 
   
   elif (F != 1.0 and RS == 0.75 and R == 1.0) or (LS != 0.75 and F == 1.0 and RS == 0.75) or (L != 1.0 and F != 1.0 and R == 1.0)  or (L != 1.0 and LS != 0.75 and R == 1.0) : 
     pub1.publish(vf*2)
     pub2.publish(vf*2)
     pub3.publish(vb)
     pub4.publish(vb) 
     
   elif (LS == 0.75 and F == 1.0 and RS != 0.75) or (L == 1.0 and RS != 0.75 and R != 1.0) or (L == 1.0 and F != 1.0 and R != 1.0) or (L == 1.0 and F != 1.0 and RS != 0.75) :
     pub1.publish(vb)
     pub2.publish(vb)
     pub3.publish(vf * 2)
     pub4.publish(vf * 2)
     
   else :
     pub1.publish(0)
     pub2.publish(0)
     pub3.publish(0)
     pub4.publish(0)
     time.sleep(0.01)
     pub1.publish(vb)
     pub2.publish(vb)
     pub3.publish(vb)
     pub4.publish(vb)
     
   rate.sleep()
rospy.spin()
