# RobotSoccer

This is the Robot Soccer repository for team Jar Jar in BYU's ECEn 490 Senior Design Project. The purpose of this repository was to create robots that would play soccer autonomously against each other in the ECEn On Display event on 4/11/2016.


To run this code, you must have ROS installed on both the robot that you wish to run it on and the vision computer that receives the vidfeed. In addition, the vidfeed computer must have OpenCV installed. We used ROS Indigo and have not tested any of the code on ROS Jade or any other versions. We also used Ubuntu Linux 14.04.1.

After you have met these prerequisites, here are the steps you should use to run the code:

1. On the vision computer, open `computer/vision` and run catkin_make clean
2. Run `catkin_make` on the same folder
3. On the robot, open `robot/motion` and then run catkin_make clean
4. Run catkin_make on the same folder
5. On the computer, add `export PATH=$PATH:~/RobotSoccer/misc_scripts/computer_setup.sh` to the last line of your .bashrc file. This is assuming the RobotSoccer folder is in your home directory. Afterwards, run `source .bashrc` in your home directory.
6. On the robot, add `export PATH=$PATH:~/RobotSoccer/misc_scripts/robot_setup.sh` to the last line of your .bashrc file. Afterwards, run `source .bashrc` in your home directory.
7. Run `computer_setup.sh` on the vision computer first, and then `robot_setup.sh` on the robot.
