from simulation import Simulation
from UR5 import UR5RobotServer
import math
import pybullet
import time

sim = Simulation()
    
sim.step_joints([0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0])
sim.add_gui_sliders()
time.sleep(1)
    
while True:
    x = pybullet.readUserDebugParameter(sim.sliders[0])
    y = pybullet.readUserDebugParameter(sim.sliders[1])
    z = pybullet.readUserDebugParameter(sim.sliders[2])
    Rx = pybullet.readUserDebugParameter(sim.sliders[3])
    Ry = pybullet.readUserDebugParameter(sim.sliders[4])
    Rz = pybullet.readUserDebugParameter(sim.sliders[5])
    position = [x,y,z]
    orientation = [Rx, Ry, Rz]  
    joint_angles = sim.calculate_ik(position, orientation)
    sim.step_joints(joint_angles)
