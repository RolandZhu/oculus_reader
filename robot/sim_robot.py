import os
import numpy as np
import pybullet as p

class RobotSim():
    initial_positions = {
        'joint_1': 0.0, 'joint_2': np.deg2rad(-33), 'joint_3': np.deg2rad(-33),
        'joint_4': 0.0, 'joint_5': np.deg2rad(-90), 'joint_6': 0.0,
        'gripper_finger_joint1': 0.025, 'gripper_finger_joint2': 0.025
    }

    def __init__(self, use_gui=True):
        self._physics_client_id = p.connect(p.GUI) if use_gui else p.connect(p.DIRECT)
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION
        self.robot_id = p.loadURDF(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fanuc/lrmate_model.urdf'),
                                   basePosition=[0.0, 0.0, 0.0], useFixedBase=True, flags=flags,
                                   physicsClientId=self._physics_client_id)
        self.end_eff_idx = 8
        
        # reset joints to home position
        self._joint_name_to_ids = {}
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self._physics_client_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i, physicsClientId=self._physics_client_id)
            joint_name = joint_info[1].decode("UTF-8")
            joint_type = joint_info[2]
            if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
                assert joint_name in self.initial_positions.keys()
                self._joint_name_to_ids[joint_name] = i

        self.debug_gui()
        self.controller_lines = []
                
    def get_joint_angle(self, eef_pos, eef_quat):
        jointPoses = p.calculateInverseKinematics(self.robot_id, self.end_eff_idx, eef_pos, eef_quat,
                                                    lowerLimits=[-3.14/2, -3.14, -3.14, -3.14, -3.14, -3.14, 0.0, 0.0],
                                                    upperLimits=[3.14/2, 3.14, 3.14, 3.14, 3.14, 3.14, 0.025, 0.025],
                                                    jointRanges=[6.28] * 6 + [0.025, 0.025],
                                                    restPoses=[0.0, 0.0, 0.0, 0.0, np.deg2rad(-90), 0.0, 0.0, 0.0],
                                                    maxNumIterations=700,
                                                    residualThreshold=.001,
                                                    physicsClientId=self._physics_client_id)
        return jointPoses
    
    def set_joints(self, joints):
        for joint_id, target in zip(list(self._joint_name_to_ids.values())[0:6], joints[0:6]):
            p.resetJointState(self.robot_id, joint_id, target, physicsClientId=self._physics_client_id)
    
    def get_eef_pose(self, joints=None):
        if joints is not None:
            self.set_joints(joints)
        state = p.getLinkState(self.robot_id, self.end_eff_idx, computeLinkVelocity=1,
                               computeForwardKinematics=1, physicsClientId=self._physics_client_id)
        pos, quat = state[0], state[1]
        return pos, quat
       
    def debug_gui(self):
        p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=-1, physicsClientId=self._physics_client_id)
        p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=-1, physicsClientId=self._physics_client_id)
        p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=-1, physicsClientId=self._physics_client_id)

        p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=self.end_eff_idx, physicsClientId=self._physics_client_id)
        p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=self.end_eff_idx, physicsClientId=self._physics_client_id)
        p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=self.end_eff_idx, physicsClientId=self._physics_client_id)
    
    def draw_controller(self, T):
        if len(self.controller_lines) > 0:
            for line in self.controller_lines:
                p.removeUserDebugItem(line, physicsClientId=self._physics_client_id)
            self.controller_lines = []
           
        line_1 = p.addUserDebugLine(T[0:3, 3], T[0:3, 3]+T[0:3, 0]*0.1, [1, 0, 0], physicsClientId=self._physics_client_id)
        line_2 = p.addUserDebugLine(T[0:3, 3], T[0:3, 3]+T[0:3, 1]*0.1, [0, 1, 0], physicsClientId=self._physics_client_id)
        line_3 = p.addUserDebugLine(T[0:3, 3], T[0:3, 3]+T[0:3, 2]*0.1, [0, 0, 1], physicsClientId=self._physics_client_id)
        self.controller_lines += [line_1, line_2, line_3]