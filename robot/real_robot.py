import socket
import numpy as np
import struct
import time

"""
This file allow Ubuntu/Mac/Windows control the robot using position/velocity.
Set your Ubuntu PC IP to 192.168.1.200, gateview to 255.255.255.1, netmask 24. (refer to https://wiki.teltonika-networks.com/view/Setting_up_a_Static_IP_address_on_a_Ubuntu_16.04_PC)
"""

class RealRobot():
    def __init__(self):
        self.UDP_IP_IN = "192.168.1.200"     # Ubuntu IP, should be the same as Matlab shows
        self.UDP_PORT_IN = 57831             # Ubuntu receive port, should be the same as Matlab shows
        self.UDP_IP_OUT = "192.168.1.100"    # Target PC IP, should be the same as Matlab shows
        self.UDP_PORT_OUT_1 = 3826           # Robot 1 receive Port
        self.UDP_PORT_OUT_2 = 3827           # Robot 2 receive Port
        self.UDP_PORT_OUT_3 = 3828           # Grippers Port

        self.s_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s_in.bind((self.UDP_IP_IN, self.UDP_PORT_IN))
        self.unpacker = struct.Struct("6d 6d 6d 6d 6d 6d")

        self.s_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.robot_1_joints, self.robot_1_vels, self.robot_1_ATI_force = None, None, None
        self.robot_2_joints, self.robot_2_vels, self.robot_2_ATI_force = None, None, None

        self.gripper = 0
        
    def receive(self):
        data, _ = self.s_in.recvfrom(1024)
        unpacked_data = np.array(self.unpacker.unpack(data))
        self.robot_1_joints, self.robot_1_vels, self.robot_1_ATI_force = unpacked_data[0:6], unpacked_data[6:12], unpacked_data[12:18]
        self.robot_2_joints, self.robot_2_vels, self.robot_2_ATI_force = unpacked_data[18:24], unpacked_data[24:30], unpacked_data[30:36]

    def send(self, target_joints_robot_1=None, target_joints_robot_2=None):
        if target_joints_robot_1 is not None:
            joint_1 = target_joints_robot_1.astype('d').tobytes()
            self.s_out.sendto(joint_1, (self.UDP_IP_OUT, self.UDP_PORT_OUT_1))
        
        if target_joints_robot_2 is not None:
            target_joints_robot_2 = target_joints_robot_2[:6]
            joint_2 = target_joints_robot_2.astype('d').tobytes()
            self.s_out.sendto(joint_2, (self.UDP_IP_OUT, self.UDP_PORT_OUT_2))

    def gripper_move(self):
        one = np.array([1])
        zero = np.array([0])
        self.s_out.sendto(one, (self.UDP_IP_OUT, self.UDP_PORT_OUT_3))
        time.sleep(0.05)
        self.s_out.sendto(zero, (self.UDP_IP_OUT, self.UDP_PORT_OUT_3))

        time.sleep(0.2)

        self.s_out.sendto(one, (self.UDP_IP_OUT, self.UDP_PORT_OUT_3))
        time.sleep(0.05)
        self.s_out.sendto(zero, (self.UDP_IP_OUT, self.UDP_PORT_OUT_3))

        if self.gripper == 0:
            self.gripper = 1
        elif self.gripper == 1:
            self.gripper = 0
        
        # time.sleep(2)

    def step(self, T=0.08):
        st = time.time()
        while time.time() - st <= T:
            self.receive()

if __name__ == '__main__':
    rc = RealRobot()

    print('home')
    target = np.array([10, 0, 0, 0, -90, 0])
    rc.send(target_joints_robot_2=target)
    rc.step(3)

    rc.gripper_move()

    # rc.gripper_move()