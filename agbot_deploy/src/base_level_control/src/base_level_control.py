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
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
import PID
from novatel_gps_msgs.msg import NovatelVelocity

global steering
global speed_set
global speed_feedback
global engineCut
global melon_location_x
global melon_location_y
global melon_detected
global object_detected
global melon_state
global melon_indexed
global trailer_state

steering = 0.0
speed_set = 0.0
speed_feedback = 0.0
engineCut = False
melon_location_x = 0
melon_location_y = 0
melon_detected = False
object_detected = "None"
# pidBypass = rospy.get_param("/pidBypass")
# melon_detection_Bypass = rospy.get_param("/melonBypass")
maxPixel = rospy.get_param("/maxPixel")
Kp_melon = rospy.get_param("/Kp_melon")
melon_state = False
melon_indexed = False
trailer_state = ""
def steeringCallback(data):
    global steering
    steering = data.data
def speedCallback(data):
    global speed_set
    speed_set = data.data
def GPSCallback(data):
    global speed_feedback
    speed_feedback = data.horizontal_speed
def engineCutCallback(data):
    global engineCut
    engineCut = data.data
def watermelon_location_Callback(data):
    global melon_location_x
    global melon_location_y
    melon_location_x = data.data[0]
    melon_location_y = data.data[1]
def melon_detected_Callback(data):
    global melon_detected
    melon_detected = data.data
def objectCallback(data):
    global object_detected
    object_detected = data.data
def trailerCallback(data):
    global trailer_state
    trailer_state = data.data

def speed_control():
    global steering
    global speed_set
    global speed_feedback
    global engineCut
    global melon_location_x
    global melon_location_y
    global melon_detected
    global object_detected
    global melon_state
    global melon_indexed
    global trailer_state
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
    rospy.Subscriber("bestvel", NovatelVelocity, GPSCallback)
    rospy.Subscriber("watermelon_location", Int16MultiArray, watermelon_location_Callback)
    rospy.Subscriber("melon_detected", Bool, melon_detected_Callback)
    rospy.Subscriber("critical_object_detected", String, objectCallback)
    rospy.Subscriber("control_state", String, trailerCallback)
    pub = rospy.Publisher('TCP_cmd', tcpCommand, queue_size = 10)
    msg = tcpCommand()
    p = PID.PID(Ki,Kp,Kd)
    p.setKi(Ki)
    p.setKd(Kd)
    p.setKp(Kp)
    p.setIMax(I_max)
    p.setIMin(I_min)
    simpleSpeedState = 1
    while not rospy.is_shutdown():
        pidBypass = int(rospy.get_param("/pidBypass_0_or_1"))
        melon_detection_Bypass = int(rospy.get_param("/melonBypass_0_or_1"))
        Kp_melon = float(rospy.get_param("/Kp_melon"))
        byPassPedalPosMax = float(rospy.get_param("/bypassPedalPosMax"))
        byPassPedalPosMin = float(rospy.get_param("/bypassPedalPosMin"))
        byPassBandWidth = float(rospy.get_param("/byPassBandWidth"))
        speed_Kp = float(rospy.get_param("/speed_Kp"))
        speed_Ki = float(rospy.get_param("/speed_Ki"))
        speed_Kd = float(rospy.get_param("/speed_Kd"))
        speed_Ki_upperLimit = int(rospy.get_param("/speed_Ki_upperLimit"))
        speed_Ki_lowerLimit = int(rospy.get_param("/speed_Ki_lowerLimit"))
        p.setKp(speed_Kp)
        p.setKi(speed_Ki)
        p.setKd(speed_Kd)
        p.setIMax(speed_Ki_upperLimit)
        p.setIMin(speed_Ki_lowerLimit)
        steering_melon = 0

        if pidBypass:
            error = speed_set - speed_feedback
            if simpleSpeedState == 1:
                msg.pedal_percent = byPassPedalPosMax
                if error > speed_set*byPassBandWidth:
                    simpleSpeedState = 2
            if simpleSpeedState == 2:
                msg.pedal_percent = (byPassPedalPosMax + byPassPedalPosMin)/2






            error = speed_set - speed_feedback
            if error < 0 - speed_set*byPassBandWidth:
                msg.pedal_percent = byPassPedalPosMin
            elif error > speed_set*byPassBandWidth:
                msg.pedal_percent = byPassPedalPosMax
            else:
                msg.pedal_percent = (byPassPedalPosMax + byPassPedalPosMin)/2
        else:
            if not speed_set == -1:

                p.setPoint(speed_set)
                pid = p.update(speed_feedback)
                if(pid > 1):
                    pid = 1
                elif pid < 0:
                    pid = 0
            msg.pedal_percent = pid
        if engineCut:
            msg.engineCut = True
        else:
            msg.engineCut = False

        if melon_detection_Bypass:
            msg.steering_percent = float(steering)

        else:
            if melon_detected and not melon_indexed:
                melon_state = True
                # melon_indexed = True
            if melon_state:
                # if melon_location_y < (maxPixel - 50):
                error = float(melon_location_x) - float(maxPixel/2)
                if abs(error) > 30:
                    steering_melon = Kp_melon * error
                    if steering_melon > 1:
                        steering_melon = 1
                    elif steering_melon < -1:
                        steering_melon = -1
                else:
                    melon_state = False
                    melon_indexed = True

            elif melon_indexed:
                steering_melon = 0
                if trailer_state == "Testing Ripeness":
                    melon_indexed = False
            else:
                steering_melon = steering
            msg.steering_percent = float(steering_melon)

        if trailer_state == "Picking up Watermelon" or \
            trailer_state == "Moving Watermelon to Bin" or \
            trailer_state == "Lowering Scooper":
            msg.pedal_percent = -1 #bypass speed control and stop the car
        if not object_detected == "None":
            msg.pedal_percent = -1
        if speed_set == -1:
            msg.pedal_percent = -1
        print(msg.pedal_percent)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    speed_control()
