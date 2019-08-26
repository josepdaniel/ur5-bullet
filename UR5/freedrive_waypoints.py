import os, inspect
from UR5 import UR5RobotServer
from pynput.keyboard import Key, Listener, KeyCode
import Simulation


finished=False
record=False

def on_release(key):
    if KeyCode.from_char("q") == key:
        print("Quit")
        global finished
        finished = True
        
    elif KeyCode.from_char("s") == key:
        global record
        record = True
        
if __name__ == "__main__":
    server=UR5RobotServer(do_handshake=False)
    listener = Listener(on_release=on_release)
    listener.start()
    
    waypoints = []
    
    print("Record waypoints by freedriving the robot and pressing 's' to save the waypoint")
    print("Stop recording waypoints by pressing 'q'")
    
    while not finished:
        if record == True:
            joints = server.get_joint_states()
            waypoints = waypoints+joints
            print("[Waypoint added]: {}".format(joints))
            record=False
            
    listener.stop()
    
    print("{} waypoints recorded.".format(len(waypoints)))
    input("Hit return to finish recording waypoints") #flush buffer
    do_sim = input("Do simulation? [y/n]: ")
    
    if do_sim == 'y':
        sim = Simulation.Simulation(camera_attached=True)
        result = sim.verify_waypoints(waypoints)

    input("Hit return to begin the program... ")
    server.connect_to_client()

    while True:
        server.serve_list(waypoints)
    

