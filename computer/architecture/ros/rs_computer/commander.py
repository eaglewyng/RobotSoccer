import rospy
from std_msgs.msg import String

print("Welcome to Team Jar Jar's commander rospy node!\n")
print("All software is copyright 2016 by Team Jar Jar\n")


print("Initializing rospy command publisher...")
pub = rospy.Publisher('commands', String, queue_size=10)
print("\nInitializing commander node...")
rospy.init_node('commander')
r = rospy.Rate(10)
print("\n\n")


while not rospy.is_shutdown()
	print("COMMAND MENU\n")
	print("------------\n")
	print("Description\tCommand\n")
	print("ramp.py\t1\n")
	print("stop.py\t2\n")
	print("\n\n")
	uinput = input("--> ")
	if uinput == "1" or uinput == "2":
		pub.publish(uinput)
		print("Command published!")
	else:
		print("Command not recognized")

	print("\n\n\n")




