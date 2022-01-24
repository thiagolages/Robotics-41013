# Robotics-41013
Assignments from 41013 Robotics subject @ University of Technology Sydney, Autumn 2019

## Assignment1 - Dual Robot Assembly Task
#### “SafeCo Assembly Task”

**Video of successful completion of the task: [link](https://www.youtube.com/watch?v=lrr2d_io8Yc)**

![image](https://user-images.githubusercontent.com/22358397/150844800-b65e7451-b079-4b71-99c5-c226cbb8d26d.png)



This task involved the collaboration between two robotics arms (Puma560) into completing a complex task. There are three parts to be
assembled, and they should be put into a specific order. In that manner, we need to know their poses and calculate joint angles for each
robot in order to pick the parts up. The trajectory in  joint-space must be smooth, the arms should move to close assembly poses in order
to perform the task, and finally put the part aside safely.

1. Pick up the circuit board with one hand
2. Pick up the bottom housing with the second hand
3. Determine an assembly pose
4. Bring the parts together, so they can be assembled
5. Pick up the top housing with the second hand
6. Bring the top housing to the assembly pose
7. Choose a place to drop the full part
8. Drop the part safely

## Assignment2 - Pick and Place Task
#### Rubbish Collector with Dobot & Turtlebot v3.0 (Extreme-Difficulty Option, check Lab Assingment 2 PDF)

**Video of successful completion of the task: [link](https://youtu.be/cC__Lanfi90)**

![image](https://user-images.githubusercontent.com/22358397/150845039-f6032b7b-8a14-407a-a055-c8512db18747.png)


This task took a lot of effort on putting two robots to work together: a mobile base to move around (Turtlebot v3.0) and a manipulator
for removing the rubbish. This took a lot of sensor integration, manipulation development and collaboration between the two robots.
The idea is that the Turtlebot will move towards a cube, representing the rubbish, with an AR tag facing the camera. It will then estimate
its relative pose with the cube, and stop at a certain distance from it. Then, it will send a signal to the Dobot to take action and remove
the rubbish* and put it aside, clearing the view for the Turtlebot and allowing it to move further.
* with its suction cap gripper

1. Move the Turtlebot until it finds the cube with an AR tag
2. Calculate its relative pose with respect with the AR tag
3. Stop moving at a certain distance from the tag
4. Send signal to the Dobot, so it can take action
5. Move the arm towards the cube
6. Activate the suction pump
7. Pute the cube aside, so it leaves clears the Turtlebot view
8. Send signal to the Turtlebot, so it can take action
9. Go back to #1

## Assignment3 - Visual Servoing & Manipulator Dynamics
#### Task 1 - Drum inspection

**Video of successful completion of the task: [link](https://youtu.be/srst6tM4fFA)**

![image](https://user-images.githubusercontent.com/22358397/150845227-647836a4-b015-41bd-a53f-f4eac3c14b2d.png)



In this task, we needed to use (Image Based) Visual Servoing to inspect 4 corners of a Drum. The task
consisted of a camera mounted on the end-effector, and it was asked to find a good position of the 
camera that allowed it to thee all the four corners. Then, we needed to inspect each one of them, 
positioning each corner on the center of the camera.

1. Mount a simulated camera on the end-effector pointing out the z-axis of the last joint
2. Manually place 4 targets on the drum
3. Find an initial pose where the circled area and targets can be seen
4. Visually servo to a closer pose where only the white plate is visible
5. Then inspect the 4 corners of the front window (the desired corner for inspection should be placed 
in the centre of the image plane). Note that when moving to and between targets we were only allowed
to use Visual Servoing techniques not Inverse Kinematics.

