import os
import itertools
from datetime import datetime
import time
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import pybullet as p
import shutil

from oculus_reader.reader import OculusReader
from robot.sim_robot import RobotSim
from robot.real_robot import RealRobot
from robot.logger import DataCollector
from camera.realsense import RealSense


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

def get_controller_diff(old_pose, new_pose):
    diff_pose = np.eye(4)
    diff_pose[0:3, 0:3] = np.linalg.inv(old_pose[0:3, 0:3]) @ new_pose[0:3, 0:3]
    diff_pose[0:3, 3] = new_pose[0:3, 3] - old_pose[0:3, 3]
    diff_pose[0:3, 3] = (np.linalg.inv(CONTROLLER_TRANSFORM) @ diff_pose[:, 3].reshape(-1, 1))[0:3].reshape(-1)
    return diff_pose

def scale_pose(diff_pose, scale):
    scaled_pose = diff_pose.copy()
    scaled_pose[0:3, 3] *= scale
    
    key_rots = R.from_matrix([np.eye(3), scaled_pose[0:3, 0:3]])
    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)
    scaled_pose[0:3, 0:3] = slerp([scale]).as_matrix()
    return scaled_pose
    
def main(max_buffer_size=5, action_scale=0.5, task='open_drawer'):
    # Init VR and robot
    time_now = datetime.now().strftime("%m%d_%H%M%S")
    save_path = f'./data/{time_now}'
    os.makedirs(save_path, exist_ok=True)
    
    try:
        vr = OculusReader()
    except:
        raise ValueError('Could not connect to VR')
    try:
        sim = RobotSim()
        real = RealRobot()
        cam = RealSense(save_path)
        logger = DataCollector(task=task, save_path=save_path)
        
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
    
    # Constants
    linked = False
    linked_pose = {'robot': np.eye(4), 'controller': np.eye(4)}
    target_joints = np.deg2rad([0, 0, 0, 0, -90, 0])
    
    # Sping the VR system for a second
    for _ in range(20):
        time.sleep(0.1)
        vr.get_transformations_and_buttons()
        real.receive()
    
    eef_pose = get_pose_matrix(*sim.get_eef_pose(np.deg2rad(real.robot_2_joints)))
    last_eef_pose = eef_pose.copy()
    
    while p.isConnected():
        last_eef_pose = eef_pose.copy()
        
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
            diff_controller_pose = get_controller_diff(linked_pose['controller'], controller_pose)
            diff_controller_pose = scale_pose(diff_controller_pose, action_scale)
            target_eef_pose = linked_pose['robot'] @ diff_controller_pose
            target_joints = sim.get_joint_angle(*get_pos_quat(target_eef_pose))
        
        elif not trigger_pressed and linked:
            linked = False
            
        elif not trigger_pressed and not linked:
            pass
        
        A_pressed = all(list(itertools.islice(A_buffer, 0, max_buffer_size)))
        if A_pressed: # Open/close gripper
            real.gripper_move()
        
        B_pressed = all(list(itertools.islice(B_buffer, 0, max_buffer_size)))
        if B_pressed: # Go home
            target_joints = np.deg2rad([0, 0, 0, 0, -90, 0])
        
        rgb_save_path = cam.get_image()
        eef_pose = get_pose_matrix(*sim.get_eef_pose(np.deg2rad(real.robot_2_joints)))
        
        # Log data
        delta_eef_pose = np.linalg.inv(last_eef_pose) @ eef_pose # EEF pose in last EEF pose
        delta_pos, delta_quat = get_pos_quat(delta_eef_pose)
        action = np.hstack([delta_pos, delta_quat])
        logger.add(rgb_save_path, real.robot_2_joints, real.robot_2_vels, real.gripper, action)
        
        # Send the command
        sim.set_joints(target_joints)
        real.send(target_joints_robot_2=np.rad2deg(target_joints))
        real.step(0.1)
        
    # Close connections and save data
    logger.save()
    shutil.rmtree(save_path)
    vr.uninstall()
    vr.stop()
    
if __name__ == '__main__':
    main()
