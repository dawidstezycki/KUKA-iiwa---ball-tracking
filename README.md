# KUKA-iiwa-ball-tracking

This repository contains the software written during internship in Robotics Lab at Cardiff University. Both programs make use of ROS and computer vision. In order to run them you need OpenCV, imutils and iiwa stack packages installed.

In the Ball Following folder you can find the files needed to make robot follow the ball's movement by using a camera attached to its gripper. You can watch it work here:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=tu50Q895ztU
" target="_blank"><img src="http://img.youtube.com/vi/tu50Q895ztU/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="360" height="270" border="10" /></a>

The software in the Ball Catching folder is meant to enable KUKA iiwa catching a ball thrown in its direction assuming it holds a bucket the ball could fall into. It employs trajectory projection to find the position the gripper should go to. Unfortunately, due to health and safety restrictions of the newly built lab, the robot couldn't run at the full speed and the frame rate of the camera used was too slow to thoroughly test the program.

Both folders include talker_zero and talker_start files. The former is used to reset the position of the robotic arm to the straight position. The latter should be run before running either of the actual programs to ensure an optimal starting position.
