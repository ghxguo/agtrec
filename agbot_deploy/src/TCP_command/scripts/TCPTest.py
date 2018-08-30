#!/usr/bin/env python
import rospy
from TCP_command.msg import tcpCommand
from socket import *
import time


pedal_per = 0.0
steering_per = 0.0
engine_cut = False

def Callback(data):
    global pedal_per
    global steering_per
    global engine_cut
    rospy.loginfo("%f, %f, %r", data.pedal_percent, data.steering_percent, data.engineCut)
    pedal_per = data.pedal_percent
    steering_per = data.steering_percent
    engine_cut = data.engineCut




def TCP_command():

    rospy.init_node("TCP_command", anonymous = True)
    rate = rospy.Rate(20)
    rospy.Subscriber("TCP_cmd", tcpCommand, Callback)
    address= ( '192.168.1.177', 5000) #define server IP and port
    client_socket =socket(AF_INET, SOCK_DGRAM) #Set up the Socket
    client_socket.settimeout(1) #Only wait 1 second for a response

    while not rospy.is_shutdown():

        steering_pos = int(steering_per * -1000)#-1000 to 1000
        pedal_pos = int(pedal_per * 450) + 1500 #1100 - 1900
        engine_cut_int = int(engine_cut)
        TXdata = "AS{0}P{1}C{2}".format(steering_pos, pedal_pos, engine_cut_int)
        client_socket.sendto(TXdata,address)
        rate.sleep()

if __name__ == '__main__':
    TCP_command()
