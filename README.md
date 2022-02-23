# Robotic
# Getting Started
We have a 3-DOF robotic arm which lengths are: L0 = L1 = L3 = 0, L2 = 2cm, L4 = 5cm, L5 = 5cm.

![robot_scr](https://user-images.githubusercontent.com/50524921/155301028-39ae9699-3ae2-4b58-9542-e86624ada4c0.png)
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

![vy](https://user-images.githubusercontent.com/50524921/69415173-d0b52800-0d1c-11ea-9828-e5878a290598.PNG)

The trajectory of the joints angles:

![joint](https://user-images.githubusercontent.com/50524921/69415280-00fcc680-0d1d-11ea-9283-8f607caccb1a.PNG)
![joint3](https://user-images.githubusercontent.com/50524921/69415482-5e911300-0d1d-11ea-9949-90efcb0ed2f2.PNG)

The trajectory of the joints speeds:

![vjoint](https://user-images.githubusercontent.com/50524921/69415593-939d6580-0d1d-11ea-9f47-52a872e6f581.PNG)
![joint3tax](https://user-images.githubusercontent.com/50524921/69415710-c8a9b800-0d1d-11ea-8f76-ea674a2becbb.PNG)

An animation of the trajectory of the robot:

![animation](https://user-images.githubusercontent.com/50524921/69416677-cf392f00-0d1f-11ea-988c-622f3fcdc33a.png)
![anim3](https://user-images.githubusercontent.com/50524921/69416728-e7a94980-0d1f-11ea-85da-680b57d65a36.png)
![anim4](https://user-images.githubusercontent.com/50524921/69416762-fe4fa080-0d1f-11ea-97e0-12121c241b78.png)



The script that compute the trajectory of robotic arm is written in Matlab.
