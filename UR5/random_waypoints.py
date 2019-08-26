from Simulation import Simulation
import random

sim = Simulation(camera_attached=True)
sim.go_to_start_pose()
sim.save_current_state()
last_successful_waypoint = []

while True:    
    waypoints = []
    # Use last good waypoint as first waypoint in next trajectory
    waypoints.extend(last_successful_waypoint) 
    
    # Generate 5 random waypoints
    for i in range(0,3):
        waypoints.append(random.uniform(0.2, 0.7))  # x
        waypoints.append(random.uniform(-0.7, 0.7)) # y
        waypoints.append(random.uniform(0.3, 0.7))  # z
        waypoints.append(random.uniform(-0.5, 0.5)) # rx
        waypoints.append(random.uniform(1.5, 1.9))  # ry
        waypoints.append(0)                         # rz

    # Simultaneously verify waypoints and calculate joint angles
    joints = sim.verify_waypoints(waypoints, use_joint_angles=False)

    if joints is -1:
        print("These waypoints caused a collision.")
        print("Resetting simulation.")
        sim.load_last_saved_state()
    else:
        print("Waypoints are good")
        last_successful_waypoint = waypoints[-6:] 
        sim.save_current_state()
    
        




