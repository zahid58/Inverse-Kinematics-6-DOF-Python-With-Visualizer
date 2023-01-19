<img src="https://github.com/Zedd1558/Inverse-Kinematics-6-DOF-for-ERC-2019/blob/master/roboticArm.png"  width="100" height="100" />

# InverseKinematics 6 DOF 
In robotics, inverse kinematics makes use of the kinematics equations to determine the joint parameters that provide a desired position for each of the robot's end-effectors. In this program, we will apply inverse kinematics on a 6 DOF robotic arm. (KUKA based design)
We have the robotic arm's measurements i.e. link lengths, link configuration, rotation information. Given coordinates X, Y, Z and roll, pitch, yaw of the end effector, We have to determine every joint angle from q1, q2, q3 ,q4, q5, q6.

### The mathematical calculations of IK are taken from https://github.com/mithi/arm-ik/

# Procedure
In the terminal enter "python armControl.py". This will open up a GUI window where you give inputs and output angles are calculated.
Inverse Kinematics is calculated in the file invKin.py. It is called by armControl.py. Arm control has a GUI that we use to control X Y Z roll pitch yaw. To see whether the output of the Inverse Kinematics is correct we use a live Matplotlib 3d graph. invKin.py has functions forwardKinematics() and getAngles() and some other helper functions. Given, the joint angles q1..q6 forwardKinematics() calculates the joint coordinates along with the end effector's coordinates. This helps us to draw the arm in matplotlib. Matplotlib window automatically pop up when you run armControl.py.
# References 
some scholarly aritcles where the algorithms is explaind in detail.\
http://scholar.google.com/scholar_url?url=https://forum.linuxcnc.org/media/kunena/attachments/1549/6-axis-serial-robot-2.pdf&hl=en&sa=X&scisig=AAGBfm0JxooSQg2Bp5iJwO5Te8mIan0TtA&nossl=1&oi=scholarr \
\
http://atmia.put.poznan.pl/Woluminy/Fil/ATMiA_34_3_5.pdf
# Some instructions
```
python armControl.py 
```
A GUI having control of X,Y,Z,roll,pitch,yaw will be initiated. A matplotlib window will be initiated. Play with GUI buttons to see inverse kinematics in action.
![](https://github.com/Zedd1558/Inverse-Kinematics-6-DOF-for-ERC-2019/blob/master/Screenshot%20(55).png)
![](https://github.com/Zedd1558/Inverse-Kinematics-6-DOF-for-ERC-2019/blob/master/Screenshot%20(54).png)
![](https://github.com/Zedd1558/Inverse-Kinematics-6-DOF-for-ERC-2019/blob/master/Screenshot%20(53).png)

# Author 
Md. Zahidul Islam,
Lecturer, CSE, IUT,
Islamic University of Technology. 
https://github.com/Zedd1558
