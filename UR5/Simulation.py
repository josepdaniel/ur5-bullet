import os
import math 
import gym
import numpy as np
import time
import pybullet 
import random
from datetime import datetime
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict

mesh_path = "../Models/meshes/objects/"
robot_urdf_path = "../ur_e_description/urdf/ur5e.urdf"
robot_with_camera_urdf_path = "../ur_e_description/urdf/ur5e_with_camera.urdf"

class Simulation(gym.Env):
  
    def __init__(self, camera_attached=False):
      
        cid = pybullet.connect(pybullet.SHARED_MEMORY)
        if(cid<0):
            print("Couldn't connect to shared memory server, using GUI")
            cid = pybullet.connect(pybullet.GUI)
    
        self.objects = self.load_scene()
        self.end_effector_index = 8
        self.start_position = [0,0,0]
        self.start_orientation = [0,0,0]
        self.start_quaternion = pybullet.getQuaternionFromEuler(self.start_orientation)
        self.table = self.load_scene()
        self.ur5 = self.load_robot(camera_attached=camera_attached)
        self.start_joint_angles = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0]
        
        base_quat = pybullet.getQuaternionFromEuler([0, 0, -math.pi])
        pybullet.resetBasePositionAndOrientation(self.ur5, [0.0, 0.0, 0.0], base_quat)
        
        pybullet.setRealTimeSimulation(1)
        
        self.control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        self.num_joints = pybullet.getNumJoints(self.ur5)
        self.joint_info = namedtuple("jointInfo", ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity", "controllable"])


        self.joints = AttrDict()
        for i in range(self.num_joints):
            info = pybullet.getJointInfo(self.ur5, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = self.joint_type_list[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = True if jointName in self.control_joints else False
            info = self.joint_info(jointID, jointName, jointType, jointLowerLimit,
                             jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            if info.type == "REVOLUTE":
                pybullet.setJointMotorControl2(self.ur5, info.id, pybullet.VELOCITY_CONTROL, targetVelocity=0, force=0)
            self.joints[info.name] = info     
        
        
    def __del__(self):
        pybullet.disconnect()
    
    def save_current_state(self):
        self.save_id = pybullet.saveState()
    
    def load_last_saved_state(self):
        pybullet.restoreState(self.save_id)
    
    def check_collisions(self):
        collisions = pybullet.getContactPoints()
        if len(collisions) > 0:
            print("[Collision detected!] {}".format(datetime.now()))
            return True
        return False
       
    
    def add_gui_sliders(self):
        self.sliders = []
        self.sliders.append(pybullet.addUserDebugParameter("X", 0, 1, 0.4))
        self.sliders.append(pybullet.addUserDebugParameter("Y", -1, 1, 0))
        self.sliders.append(pybullet.addUserDebugParameter("Z", 0.3, 1, 0.4))
        self.sliders.append(pybullet.addUserDebugParameter("Rx", -math.pi/2, math.pi/2, 0))
        self.sliders.append(pybullet.addUserDebugParameter("Ry", -math.pi/2, math.pi/2, 0))
        self.sliders.append(pybullet.addUserDebugParameter("Rz", -math.pi/2, math.pi/2, 0))
        
    def load_scene(self):
        table = [pybullet.loadURDF((os.path.join(pybullet_data.getDataPath(), "table/table.urdf")), 0.5, 0.0, -0.6300, 0.000000, 0.000000, 0.0, 1.0)]
        return [table]
        
    def load_robot(self, camera_attached):
        if camera_attached:
            return pybullet.loadURDF(robot_with_camera_urdf_path, self.start_position, self.start_quaternion, 
                                     flags=pybullet.URDF_USE_INERTIA_FROM_FILE|pybullet.URDF_USE_SELF_COLLISION)
        else:
            return pybullet.loadURDF(robot_urdf_path, self.start_position, self.start_quaternion, 
                                     flags=pybullet.URDF_USE_INERTIA_FROM_FILE|pybullet.URDF_USE_SELF_COLLISION)

    def calculate_ik(self, position, orientation):
        quaternion = pybullet.getQuaternionFromEuler(orientation)
        
        lower_limits = [-math.pi]*6
        upper_limits = [math.pi]*6
        joint_ranges = [2*math.pi]*6
        rest_poses = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0]
        joint_angles = pybullet.calculateInverseKinematics(self.ur5, self.end_effector_index, position, quaternion, 
                                                           jointDamping=[0.01]*6, upperLimits=upper_limits, 
                                                           lowerLimits=lower_limits, jointRanges=joint_ranges, 
                                                           restPoses=rest_poses)
        return joint_angles
        
    def get_current_joint_angles(self):
        j = pybullet.getJointStates(self.ur5, [1,2,3,4,5,6])
        joints = [i[0] for i in j]
        return joints
    
    def step_joints(self, joint_angles):
        poses = []
        indexes = []
        forces = []

        for i,name in enumerate(self.control_joints):
            joint = self.joints[name]
            poses.append(joint_angles[i])
            indexes.append(joint.id)
            forces.append(joint.maxForce)

        pybullet.setJointMotorControlArray(self.ur5, indexes,
                                           pybullet.POSITION_CONTROL,
                                           targetPositions=joint_angles,
                                           targetVelocities = [0]*len(poses),
                                           positionGains = [0.005]*len(poses), forces = forces)
    
    def run(self):
        self.go_to_start_pose()
        self.add_gui_sliders()
        while True:
            x = pybullet.readUserDebugParameter(self.sliders[0])
            y = pybullet.readUserDebugParameter(self.sliders[1])
            z = pybullet.readUserDebugParameter(self.sliders[2])
            Rx = pybullet.readUserDebugParameter(self.sliders[3])
            Ry = pybullet.readUserDebugParameter(self.sliders[4])
            Rz = pybullet.readUserDebugParameter(self.sliders[5])
            position = [x,y,z]
            orientation = [Rx, Ry, Rz]
            joint_angles = self.calculate_ik(position, orientation)
            self.step_joints(joint_angles)
            self.check_collisions()

    def go_to_start_pose(self):
        self.step_joints(self.start_joint_angles)
        error = self.calculate_error(self.start_joint_angles)
        while(error>0.04):
            error = self.calculate_error(self.start_joint_angles)
    
    def calculate_error(self, goal):
        current = self.get_current_joint_angles()
        error = sum([abs(i[0]-i[1]) for i in zip(goal, current)])
        return error
    
    def run_waypoints(self, waypoints):
        """ Loop through waypoints infinitely """
        i=0
        joints = waypoints[0:6]
        self.step_joints(joints)
        while True:
            self.check_collisions()
            error = self.calculate_error(joints)
            if error <= 0.04:
                i = i+6 if i<len(waypoints)-6 else 0
                print("Waypoint {}".format(i/6))
                joints = waypoints[i:i+6]
                self.step_joints(joints)

    def verify_waypoints(self, waypoints, use_joint_angles=True):
        """ 
        Returns completed joint angles if waypoints are good, -1 if waypoints cause collision.
        Specify use_joint_angles=False if passing in position/rotation vectors, leave as True
        if passing in joint angles. 
        """
        
        i=0
        joints_list = []
        num_waypoints = len(waypoints)
        waypoints.extend(waypoints) # repeat trajectory twice
        
        joints = waypoints[0:6]
        if not use_joint_angles: 
            joints = self.calculate_ik(joints[0:3], joints[-3:])
        
        joints_list.extend(joints)
        self.step_joints(joints)
        print("Waypoint {}: {}".format(i/6, joints))
        
        while True:
            collides = self.check_collisions()
            if collides is True: return -1 
            error = self.calculate_error(joints)
            if error <= 0.04:
                i = i+6    
                if i > len(waypoints) - 6: return joints_list[0:num_waypoints]
                joints = waypoints[i:i+6]
                if not use_joint_angles: 
                    joints = self.calculate_ik(joints[0:3], joints[-3:])
                    print("{}, {}".format(joints[0:3], joints[-3:]))
                print("Waypoint {}: {}".format(i/6, joints))
                joints_list.extend(joints)
                self.step_joints(joints)

           
if __name__ == "__main__":
    sim = Simulation()
    
    sim.step_joints([0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0])
    time.sleep(4)
    
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


        
        
        
        
