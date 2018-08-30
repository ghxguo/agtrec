#!/usr/bin/env python
#The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
#######	Example	#########
#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#
import rospy
from TCP_command.msg import tcpCommand
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import PID

global steering
global speed_set
global speed_feedback
global engineCut
steering = 0.0
speed_set = 0.0
speed_feedback = 0.0
engineCut = False
pidBypass = rospy.get_param("/pidBypass")

def steeringCallback(data):
    global steering
    steering = data.data
def speedCallback(data):
    global speed_set
    speed_set = data.data
def GPSCallback(data):
    global speed_feedback
    speed_feedback = data.data
def engineCutCallback(data):
    global engineCut
    engineCut = data.data

def speed_control():
    global steering
    global speed_set
    global speed_feedback
    global engineCut
    Kp = 0.2
    Ki = 0.1
    Kd = 3.0
    I_max = 100
    I_min = -100
    accel_max = 1
    rospy.init_node("speed_control", anonymous = True)
    rate = rospy.Rate(20)
    rospy.Subscriber("speed_setpoint", Float32, speedCallback)
    rospy.Subscriber("steering_cmd", Float32, steeringCallback)
    rospy.Subscriber("engine_cut", Bool, engineCutCallback)
    rospy.Subscriber("GPS_speed", Float32, GPSCallback)
    pub = rospy.Publisher('TCP_cmd', tcpCommand, queue_size = 10)
    msg = tcpCommand()
    p = PID.PID(Ki,Kp,Kd)
    p.setKi(Ki)
    p.setKd(Kd)
    p.setKp(Kp)
    p.setIMax(I_max)
    p.setIMin(I_min)
    while not rospy.is_shutdown():
        msg.steering_percent = steering
        p.setPoint(speed_set)
        pid = p.update(speed_feedback)
        if(pid > 1):
            pid = 1
        if pidBypass:
            msg.pedal_percent = speed_set
        else:
            msg.pedal_percent = pid
        if engineCut:
            msg.engineCut = True
        else:
            msg.engineCut = False
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    speed_control()
