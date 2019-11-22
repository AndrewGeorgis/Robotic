# Robotic
# Getting Started
We have a 3-DOF robotic arm which lengths are: L0 = L1 = L3 = 0, L2 = 2cm, L4 = 5cm, L5 = 5cm.

![3dof-robotpng](https://user-images.githubusercontent.com/50524921/69413478-8da58580-0d19-11ea-81d8-65de60b10b9a.png)

# Step 1
Compute forward and inverse kinematic of the robot arm.

# Step 2 
Given that the robot perform a straight periodic move between the points A(2,5,3) and B(8,-5,3) with period T = 10sec, the goal is to compute and design the trajectory of the robot with smooth speeds and no oscillations in the joints.To accomplish the smoothness for the speeds, we choose a 3 degree polynomial for the trajectory of the robot. Above we plot the movement of the x-axis, y-axis for the end effector,as the trajectory of the speeds at x-axis, y-axis of the end effector. Moreover we show the smoothness of the speeds of the joints.

At the x-axis trajectory

![θεση](https://user-images.githubusercontent.com/50524921/69414755-04438280-0d1c-11ea-9958-c98c51ded27a.PNG)

At the y-axis trajectory

![yθεση](https://user-images.githubusercontent.com/50524921/69414850-32c15d80-0d1c-11ea-9177-5efd48057df4.PNG)

The linear speed of the end effector at the x-axis:

![vx](https://user-images.githubusercontent.com/50524921/69415035-8df35000-0d1c-11ea-98f2-e51597745658.PNG)

The linear speed of the end effector at the y-axis:

