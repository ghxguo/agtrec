#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
#from TCP_command.msg import tcpCommand


left_vertical = 0.0
right_horizontal = 0.0
engine_cut = False


def callback(data):
    global left_vertical
    global right_horizontal
    global engine_cut
    rospy.loginfo("%f, %f", data.axes[1], data.axes[2])
    left_vertical = data.axes[1]
    right_horizontal = data.axes[2]
    if data.axes[18] == -1:
        engine_cut = True
    else:
        engine_cut = False


def joyTranslate():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    #pub_left = rospy.Publisher('pedal_pos', Float32, queue_size = 10)
    #pub_right = rospy.Publisher('steering_pos', Float32, queue_size = 10)
    pub_st = rospy.Publisher('steering_cmd', Float32, queue_size = 10)
    pub_pd = rospy.Publisher('speed_setpoint', Float32, queue_size = 10)
    pub_ec = rospy.Publisher('engine_cut', Bool, queue_size = 10)
    rospy.init_node('joyTranslate', anonymous=True)
    rate = rospy.Rate(20)
    rospy.Subscriber("joy", Joy, callback)
    #msg = tcpCommand()
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    while not rospy.is_shutdown():
        #pub_left.publish(left_vertical)
        #pub_right.publish(right_horizontal)
        #msg.pedal_percent = left_vertical
        pub_pd.publish(left_vertical)
        pub_st.publish(right_horizontal)
        #msg.steering_percent = right_horizontal
        pub_ec.publish(engine_cut)
        rate.sleep()

if __name__ == '__main__':
    joyTranslate()
