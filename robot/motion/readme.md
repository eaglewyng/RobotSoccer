#Robot code
##Folder organization
`Datasheets` contains helpful information on the roboclaws and other devices.

`src/motion/scripts` contains the bulk of the code, including the main script that is used to run the robot's strategy (Vektory.py) and the CommandCenter which shows the robots' locations on the computer and sends commands via CommandNode (VektoryCommandCenter.py)

`src/motion/msg` contains the messages broadcasted from the ROS topics that are essential for communication, including locations.msg and velocities.msg

Many of the service messages in `src/motion/srv` are depricated since we moved our robot architecture from service-based communication to topic-based communication.
