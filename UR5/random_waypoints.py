from Simulation import Simulation
import random
import math
import sys
import itertools

sim = Simulation(camera_attached=True)
sim.go_to_start_pose()
sim.save_current_state()
last_successful_waypoint = []

def grouper(n, iterable):
    it = iter(iterable)
    while True:
       chunk = tuple(itertools.islice(it, n))
       if not chunk:
           return
       yield chunk

def generate_random_waypoint():
    waypoints = []
    #Pick a random point on the surface of a unit hemi-sphere
    elevation = random.uniform(0.8, math.pi)
    azimuth = random.uniform(-2.6, 2.6)
    radius = random.uniform(0.5, 0.8)
    # Calculate point in 3d space
    x = radius * math.cos(elevation) * math.cos(azimuth)+0.2
    y = radius * math.cos(elevation) * math.sin(azimuth)+0.2
    z = radius * math.sin(elevation)
    # Add boundaries
    if(x>0.8): x=0.8
    if(x<0.35): x =0.35
    if(y>0.7): y=0.7
    if(y<-0.7): y=-0.7
    if(z>0.7): z=0.7
    if(z<0.3): z=0.3
    # Uniform RPY
    roll = 0
    pitch = 0
    yaw = 0
    return [x,y,z,roll,pitch,yaw]
    

if len(sys.argv) > 1:
    outfile = open(sys.argv[1], 'w+')

for _ in range(0,5):
    waypoints = []
    # Use last good waypoint as first waypoint in next trajectory
    waypoints.extend(last_successful_waypoint) 

    # Generate 5 random waypoints
    for i in range(0,5):
        waypoints.extend(generate_random_waypoint())
       
    # Simultaneously verify waypoints and calculate joint angles
    joints = sim.verify_waypoints(waypoints, use_joint_angles=False)

    if joints is -1:
        print("These waypoints caused a collision.")
        print("Resetting simulation and generating new waypoints.")
        sim.load_last_saved_state()
    else:
        print("Waypoints are good!")
        if len(sys.argv) > 1:
            if(len(last_successful_waypoint) > 0):
                joint_states_to_record = joints[6:]
            else:
                joint_states_to_record = joints
            
            for waypoint in grouper(6, joint_states_to_record):
                outfile.write("{},{},{},{},{},{}".format(waypoint[0],waypoint[1],waypoint[2],waypoint[3],waypoint[4],waypoint[5]))
                outfile.write("\n")
        last_successful_waypoint = waypoints[-6:] 
        sim.save_current_state() # If simulation fails, we can load back to this state

if len(sys.argv) > 1:
    outfile.close()
    
        





