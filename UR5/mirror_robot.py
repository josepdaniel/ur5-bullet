from UR5 import UR5RobotServer
import Simulation
import time
import pybullet


"""
Display on the screen the actual pose of the real robot
"""

server = UR5RobotServer(do_handshake=False)
sim = Simulation.Simulation()

# time.sleep(0.5)

while(True):
    joints = server.get_joint_states()
    # print("joints are {}".format(joints))
    if joints is None:
        pass
    else:
        sim.step_joints(joints)
        #collisions = pybullet.getContactPoints()
        #if (len(collisions) > 0):
        #    print("Collision detected by pybullet!")

