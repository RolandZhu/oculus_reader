import os
import itertools
import time
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R

from oculus_reader.reader import OculusReader
from robot.sim_robot import RobotSim

CONTROLLER_TRANSFORM = np.array([[0, -1, 0, 0],
                                 [0, 0, -1, 0],
                                 [1, 0, 0, 0],
                                 [0, 0, 0, 1]])

def get_pose_matrix(pos, quat):
    T = np.eye(4)
    T[0:3, 0:3] = R.from_quat(quat).as_matrix()
    T[0:3, 3] = pos
    return T

def get_pos_quat(T):
    return T[0:3, 3], R.from_matrix(T[0:3, 0:3]).as_quat()
    
def main(max_buffer_size=5):
    try:
        vr = OculusReader()
    except:
        raise ValueError('Could not connect to VR')
    try:
        sim = RobotSim()
    except:
        raise ValueError('Could not connect to robot')
    
    # Buffer for right controller trigger button, A button, and pose
    trigger_buffer = deque(maxlen=max_buffer_size)
    A_buffer = deque(maxlen=max_buffer_size)
    B_buffer = deque(maxlen=max_buffer_size)
    pose_buffer = deque(maxlen=max_buffer_size)
    for _ in range(max_buffer_size):
        trigger_buffer.append(False)
        A_buffer.append(False)
        B_buffer.append(False)
        pose_buffer.append(np.eye(4))
    
    linked = False
    linked_pose = {'robot': np.eye(4), 'controller': np.eye(4)}
    
    target_joints = np.deg2rad([0, 0, 0, 0, -90, 0])
    
    while True:
        time.sleep(0.1)
        
        # Log controller data
        transforms, buttons = vr.get_transformations_and_buttons()
        trigger_buffer.append(buttons['RTr'])
        A_buffer.append(buttons['A'])
        B_buffer.append(buttons['B'])
        pose_buffer.append(transforms['r'])
        
        controller_pose = transforms['r'] @ CONTROLLER_TRANSFORM
        
        trigger_pressed = all(list(itertools.islice(trigger_buffer, max_buffer_size-3, max_buffer_size)))
        
        if trigger_pressed and not linked:
            # If the trigger is pressed and not linked, link the robot to the controller
            linked = True
            eef_pose = sim.get_eef_pose()
            linked_pose['robot'] = get_pose_matrix(*eef_pose)
            linked_pose['controller'] = controller_pose
        elif trigger_pressed and linked:
            # If the trigger is pressed and linked, move the robot based on controller movement
            diff_controller_pose = np.linalg.inv(linked_pose['controller']) @ controller_pose
            target_eef_pose = linked_pose['robot'] @ diff_controller_pose
            target_joints = sim.get_joint_angle(*get_pos_quat(target_eef_pose))
        
        elif not trigger_pressed and linked:
            linked = False
        elif not trigger_pressed and not linked:
            pass
        
        A_pressed = all(list(itertools.islice(A_buffer, 0, max_buffer_size)))
        
        B_pressed = all(list(itertools.islice(B_buffer, 0, max_buffer_size)))
        if B_pressed: # Go home
            target_joints = np.deg2rad([0, 0, 0, 0, -90, 0])
        
        sim.set_joints(target_joints)
    

if __name__ == '__main__':
    main()
