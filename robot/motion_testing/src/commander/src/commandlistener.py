#!/usr/bin/python
import subprocess
import rospy
from std_msgs.msg import String

import ramp

def callback(data):
    if data.data == "1":
        print("Command \"1\" received! Starting ramp.py...\n")
        ramp.singleramp()
    elif data.data == "2":
        print("Command \"2\" received! Starting stop.py...\n")
        import stop
    else:
        print("Command \"" + data.data + "\" unrecognized\n")
    rospy.loginfo("I heard %s",data.data)

def listener():
    print("I be listenin'")
    rospy.init_node('commandlistener')
    rospy.Subscriber("commands", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()
