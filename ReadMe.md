run armControl.py
it will import invKin.py, SerialCom.py

# InverseKinematics - problem formulation
In robotics, inverse kinematics makes use of the kinematics equations to determine the joint parameters that provide a desired position for each of the robot's end-effectors. In this program, we will apply inverse kinematics on a 6 DOF robotic arm. (KUKA based design)
We have the robotic arm's measurements i.e. link lengths, link configuration, rotation information. Given coordinates X, Y, Z and roll, pitch, yaw of the end effector, We have to determine every joint angle from q1, q2, q3 ,q4, q5, q6.
# Procedure
Inverse Kinematics is solved by invKin.py. It is called by armControl.py. Arm control has a GUI that we use to control X Y Z roll pitch yaw. To see whether the output of the Inverse Kinematics is correct we use a live Matplotlib 3d graph. invKin.py has functions forwardKinematics() and getAngles() and some other helper functions. Given, the joint angles q1..q6 forwardKinematics() calculates the joint coordinates along with the end effector's coordinates. This helps us to draw the arm in matplotlib. Matplotlib window automatically run when armControl.py is run.
# References 
some scholarly aritcles where the scenerio is explaind in detail.
http://scholar.google.com/scholar_url?url=https://forum.linuxcnc.org/media/kunena/attachments/1549/6-axis-serial-robot-2.pdf&hl=en&sa=X&scisig=AAGBfm0JxooSQg2Bp5iJwO5Te8mIan0TtA&nossl=1&oi=scholarr \\
http://atmia.put.poznan.pl/Woluminy/Fil/ATMiA_34_3_5.pdf
# Some instructions
run armControl.py. A GUI having control of X,Y,Z,roll,pitch,yaw will be initiated. A matplotlib window will be initiated. Play with GUI buttons to see inverse kinematics in action.
![](https://github.com/iut-160041010/InverseKinematicsERC19/blob/master/Applying%20Inverse%20Kinematics/Slide2.JPG)
# Author 
Md. Zahidul Islam,
Undergrad, CSE, IUT,
Islamic University of Technology.
