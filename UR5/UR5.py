from math import pi
import itertools
import socket
import time
import threading
import codecs
import struct
import math

class UR5RobotServer():

    def __init__(self, host_ip="129.78.210.145", port=30000, do_handshake=True):
        self.HOST = host_ip
        self.PORT = port
        self.ROBOT_IP = "129.78.214.100"
        self.ROBOT_PORT = 30003
    
        self.max_msg_length = 90
    
        self.real_pose = None
        self.real_joint_states = None
        
        self._lock = threading.Lock()
        self.feedback_thread = None

        self.feedback_thread = threading.Thread(target=self.feedback)
        self.feedback_thread.start()
        self.connection = None
        if do_handshake:
            self.connection = self.handshake()

    def __del__(self):
        if self.feedback_thread is not None:
            self.feedback_thread.join()
        if self.connection is not None:
            self.connection.close()

    def handshake(self):
        """ 
        Synchronous handshake with UR5, returns once handshake is complete.
        Returns a socket object that can be read and written to   
        """
        print("Waiting for handhsake...")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))  
        s.listen(5)
        c, addr = s.accept()

        while True:
            msg = c.recv(1024).decode('utf-8')
            if msg == "Shaking hands":
                print("[Rx] {}".format(msg))
                print("Handshake complete.")
                return c
                
    def wait_for_rx_message(self, expected_message):
        while True:
            msg = self.connection.recv(1024).decode('utf-8')
            print("Waiting for rx message")
            if msg == expected_message:
                print("[Rx] {}".format(msg))
                break
    
    def connect_to_client(self):
        print("Doing handshake with robot. Ensure program is running on the client before proceeding.")
        self.connection = self.handshake()
        
    def serve_list(self, tx_list):
        """
        Convert list to bytes and serves to robot. Robot only accepts list of length 30,
        so if the list length is greater than 30, it will be broken into chunks. 
        """
        if len(tx_list)>30:
            print("Message will be split into chunks of 30")
            for chunk in grouper(30, tx_list):
                tx_msg = bytes("{}".format(chunk), 'utf-8')
                self.serve_bytes(tx_msg)
                print("[Tx] {}".format(tx_msg))
                self.wait_for_rx_message("ready")
                    # time.sleep(1)
                
        else:
            tx_msg = bytes("{}".format(tx_list), 'utf-8')
            self.serve_bytes(tx_msg)
            print("[Tx] {}".format(tx_msg))   
            self.wait_for_rx_message("ready")         


    def serve_bytes(self, msg):
        """
        Sends msg over TCP connection.
        """
        try:
            self.connection.send(msg)
        except socket.error as socketerror:
            print("Failed to transmit bytes to UR5.")

    def get_joint_states(self):
        return self.real_joint_states

    def feedback(self):
        """
        Constantly reads feedback data from robot
        """
        while True:
            try:
                measured_joint_angles = []
                recieve_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                recieve_socket.settimeout(10)
                recieve_socket.connect((self.ROBOT_IP, self.ROBOT_PORT))

                # GARBAGE
                packet_1 = recieve_socket.recv(4)
                packet_2 = recieve_socket.recv(8)
                packet_3 = recieve_socket.recv(48)
                packet_4 = recieve_socket.recv(48)
                packet_5 = recieve_socket.recv(48)
                packet_6 = recieve_socket.recv(48)
                packet_7 = recieve_socket.recv(48)
                    
                    # JOINT ANGLES
                packet_8a = recieve_socket.recv(8)
                packet_8a = codecs.encode(packet_8a, "hex")
                measured_joint_angles.append(struct.unpack('!d', codecs.decode(packet_8a, "hex"))[0])
                packet_8b = recieve_socket.recv(8)
                packet_8b = codecs.encode(packet_8b, "hex")
                measured_joint_angles.append(struct.unpack('!d', codecs.decode(packet_8b, "hex"))[0])
                packet_8c = recieve_socket.recv(8)
                packet_8c = codecs.encode(packet_8c, "hex")
                measured_joint_angles.append(struct.unpack('!d', codecs.decode(packet_8c, "hex"))[0])
                packet_8d = recieve_socket.recv(8)
                packet_8d = codecs.encode(packet_8d, "hex")
                measured_joint_angles.append(struct.unpack('!d', codecs.decode(packet_8d, "hex"))[0])
                packet_8e = recieve_socket.recv(8)
                packet_8e = codecs.encode(packet_8e, "hex")
                measured_joint_angles.append(struct.unpack('!d', codecs.decode(packet_8e, "hex"))[0])
                packet_8f = recieve_socket.recv(8)
                packet_8f = codecs.encode(packet_8f, "hex")
                measured_joint_angles.append(struct.unpack('!d', codecs.decode(packet_8f, "hex"))[0])
                self.real_joint_states = measured_joint_angles
                # print("Measured joint angles: {}".format([round(x,2) for x in self.real_joint_states]))

                # GARBAGE
                packet_9 = recieve_socket.recv(48)
                packet_10 = recieve_socket.recv(48)
                packet_11 = recieve_socket.recv(48)
                packet_12 = recieve_socket.recv(8)

                # POSE
                packet_12 = codecs.encode(packet_12, "hex") #convert the data from \x hex notation to plain hex
                x = str(packet_12)
                x = struct.unpack('!d', codecs.decode(packet_12, "hex"))[0] 
                packet_13 = recieve_socket.recv(8)
                packet_13 = codecs.encode(packet_13, "hex") #convert the data from \x hex notation to plain hex
                y = str(packet_13)
                y = struct.unpack('!d', codecs.decode(packet_13, "hex"))[0]
                packet_14 = recieve_socket.recv(8)
                packet_14 = codecs.encode(packet_14, "hex") #convert the data from \x hex notation to plain hex
                z = str(packet_14)
                z = struct.unpack('!d', codecs.decode(packet_14, "hex"))[0]
                packet_15 = recieve_socket.recv(8)
                packet_15 = codecs.encode(packet_15, "hex") #convert the data from \x hex notation to plain hex
                Rx = str(packet_15)
                Rx = struct.unpack('!d', codecs.decode(packet_15, "hex"))[0]
                packet_16 = recieve_socket.recv(8)
                packet_16 = codecs.encode(packet_16, "hex") #convert the data from \x hex notation to plain hex
                Ry = str(packet_16)
                Ry = struct.unpack('!d', codecs.decode(packet_16, "hex"))[0]
                packet_17 = recieve_socket.recv(8)
                packet_17 = codecs.encode(packet_17, "hex") #convert the data from \x hex notation to plain hex
                Rz = str(packet_17)
                Rz = struct.unpack('!d', codecs.decode(packet_17, "hex"))[0]
                position = [x,y,z]
                euler = [Rx, Ry, Rz]
                # print("Measured position and orientation: {}".format([round(x,2) for x in position+euler]))
                    
                recieve_socket.close()
     
            except socket.error as socketerror:
                print("Error: ", socketerror)
                    
           
         
def grouper(n, iterable):
    it = iter(iterable)
    while True:
       chunk = tuple(itertools.islice(it, n))
       if not chunk:
           return
       yield chunk
       

       
if __name__ == "__main__":
    server = UR5RobotServer()
    
    
    
    waypoint1 = [11.16, -118.32, -95.01, -328.21, 84.47, 87.86]
    waypoint2 = [11.16, -109.51, -90.23, -340.60, 84.47, 87.86]
    waypoint3 = [7.73, -96.89, -89.95, -355.87, 84.47, 87.85]
    waypoint4 = [-1.44, -94.18, -88.74, -355.94, 96.92, 87.85]
    waypoint5 = [-15.76, -99.88, -79.76, -356.32, 112.23, 87.86]
    waypoint6 = [-20.32, -111.60, -85.75, -342.22, 112.21, 87.85]
    
    waypoints = [math.radians(x) for x in waypoint1+waypoint6+waypoint4+waypoint5+waypoint1+waypoint2+waypoint3+waypoint4+waypoint5]

    server.serve_list(waypoints)







    

