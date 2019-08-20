# ArmControl

## About
This branch implements some trajectory control programs for the UR5. For position/orientation control over the robot, use the Jesse branch.

## UR5RobotServer class
This python3 class handles communication with the UR5, and assumes that a client program is running on the robot. To instantiate a server instance:

```python
server=UR5RobotServer(host_ip=IP-ADDRESS-OF-PC, port=30000, do_handshake=True)
```
The do_handshake parameter informs the robot that the PC is going to begin sending waypoints. Without performing the handshake, waypoints received by the robot will not be executed. The do_handshake parameter can be set to False if the program simply wishes to read feedback data from the robot (e.g. if we want to free-drive the robot, or control its position with the teaching pendant). 

There are two communication streams between the robot and the PC. The client-server communication stream enables sending waypoints (represented as a list of 6 joint angles) to the robot, while the second communication stream constantly reads feedback joint position data from the robot (it is strangely known as MATLAB data). The client-server stream is only instantiated after the handshake has been performed, whereas the second communication stream is constantly active, even before the handshake occurs. If you want to simply read joint angles from the robot, it is best to set do_handshake to False, as it is a blocking function call. 

Run ```server.connect_to_client()``` to explicitly perform the handshake if the object was instantiated with do_handshake=False. 

The advantage of using a client-server model for controlling the arm is that all safety features of the polyscope pendant will flow through to the program execution itself. For example, if we set the tool speed to 50% on the pendant, the program will execute at 50% speed. We can even change the trajectory speed in real-time, while the program is executing. In contrast, using URScript to control the arm remotely, the user is responsible for setting the tool speed for every movement, and this speed cannot be changed once the command has been received. Furthermore, URScript commands sent via the TCP connection are non-blocking. Even with well timed commands, experimenting showed that fine-grained trajectories would often result in jerky movements, which is unfavourable for smooth video-capture. 

## Smooth trajectory between waypoints using freedrive

In the UR5 directory run:

```python
python3 freedrive_waypoints.py
```
On the teaching pendant, run the program called remote_control2.script.

You will be prompted to freedrive the robot (by pressing the small black button in the center of the top of the pendant). At each desired waypoint, hit the 's' key on the PC keyboard to save the waypoint, or press 'q' to signal that you have recorded all desired waypoints. You can then choose to either run a coarse-trajectory simulation between your chosen waypoints running in the pybullet physics engine, or proceed directly to running the program on the robot.  
After the client (robot) and server (PC) successfully perform a handshake, the robot should begin to follow the recorded waypoints, smoothed by a blend radius of 0.1m. 

Demo:
https://drive.google.com/open?id=13Uj1Ezqt68gSXO6k_RxfVklmd2PciOR7

### Current Limitations
- Since the blend radius is hard-coded as 0.1m, some moves will fail to interpolate a path, e.g. a direct reversal of trajectory. Solution: investigate dynamically calculating blend radii. 
- If the same waypoint is received twice consecutively, the robot will raise an error.
- The program does not exit graciously - use ctrl-c to exit on server-side, use the stop button to exit on the teaching pendant.


## Mirror the physical robot in simulation

In the UR5 directory run:

```python
python3 mirror_robot.py
```

A simulated robot will appear on the screen which mimics the real robot. You can control the robot using the teaching pendant or freedrive it. This is a useful demonstration of the ```do_handshake=False``` parameter. In this case we are reading off the joint data from the robot, but we don't want to handshake with the robot because we will not be sending any waypoints. It is also a useful check that collision detection is working. 


## Control a simulated robot
In the UR5 directory run:

```python
python3 control_sim.py
```




## Resources
Simulation and UR5 originally written by Sholto Douglas 
https://github.com/sholtodouglas/ur5pybullet

Simulation using pybullet 
https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914.pdf

URScript API
http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf

Reading notes on client-server side communication with the arm
https://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/universal-robots-script-programming/