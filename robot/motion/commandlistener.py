import subprocess
import rospy
from std_msgs.msg import String

def callback(data):
	if data.data == "1":
		print("Command \"1\" received! Starting ramp.py...\n")
		subprocess.call("ramp.py")
	elif data.data == "2":
		print("Command \"2\" received! Starting stop.py...\n")
		subprocess.call("stop.py")
	else:
		print("Command \"" + data.data + "\" unrecognized\n")

    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('commandlistener')
    rospy.Subscriber("commands", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()