#!/usr/bin/env python3
"""
Unitree G1 Robot Dance Sequence
Classic robot dance with mechanical movements and poses
"""

import numpy as np
import math

class RobotDance:
    def __init__(self):
        self.bpm = 100  # Slower, more mechanical tempo
        self.beat_duration = 60.0 / self.bpm
        
        self.joints = {
            'left_hip_yaw': 0, 'left_hip_roll': 1, 'left_hip_pitch': 2,
            'right_hip_yaw': 3, 'right_hip_roll': 4, 'right_hip_pitch': 5,
            'left_shoulder_pitch': 6, 'left_shoulder_roll': 7, 'left_elbow': 8,
            'right_shoulder_pitch': 9, 'right_shoulder_roll': 10, 'right_elbow': 11,
            'waist': 12
        }
    
    def mechanical_step(self, t):
        """Sharp, angular stepping"""
        beat_num = int(t / self.beat_duration) % 4
        if beat_num == 0:
            return 0.3, 0  # Left foot forward
        elif beat_num == 1:
            return 0, 0    # Center
        elif beat_num == 2:
            return 0, 0.3  # Right foot forward
        else:
            return 0, 0    # Center
    
    def robot_arms(self, t):
        """Classic robot arm movements - 90 degree angles"""
        beat_phase = (t / self.beat_duration) % 8
        
        if beat_phase < 2:  # Arms up
            l_shoulder = -1.57  # 90 degrees up
            r_shoulder = -1.57
            l_elbow = 1.57      # 90 degree bend
            r_elbow = 1.57
        elif beat_phase < 4:  # Arms forward
            l_shoulder = 0
            r_shoulder = 0
            l_elbow = 1.57
            r_elbow = 1.57
        elif beat_phase < 6:  # Arms to sides
            l_shoulder = 0
            r_shoulder = 0
            l_elbow = 0
            r_elbow = 0
        else:  # Arms crossed
            l_shoulder = 0.5
            r_shoulder = -0.5
            l_elbow = 1.57
            r_elbow = 1.57
        
        return l_shoulder, r_shoulder, l_elbow, r_elbow
    
    def head_nod(self, t):
        """Mechanical head nodding"""
        nod_phase = (t / self.beat_duration) % 2
        if nod_phase < 1:
            return 0.3  # Nod down
        else:
            return -0.2  # Nod up
    
    def generate_sequence(self, duration=40):
        sequence = []
        dt = 0.02
        
        for i in range(int(duration / dt)):
            t = i * dt
            left_step, right_step = self.mechanical_step(t)
            l_shoulder, r_shoulder, l_elbow, r_elbow = self.robot_arms(t)
            
            # Sharp waist turns
            waist_turn = 0.5 * (1 if int(t / (2 * self.beat_duration)) % 2 == 0 else -1)
            
            joint_cmd = np.zeros(len(self.joints))
            joint_cmd[self.joints['left_hip_pitch']] = left_step
            joint_cmd[self.joints['right_hip_pitch']] = right_step
            joint_cmd[self.joints['left_shoulder_pitch']] = l_shoulder
            joint_cmd[self.joints['right_shoulder_pitch']] = r_shoulder
            joint_cmd[self.joints['left_elbow']] = l_elbow
            joint_cmd[self.joints['right_elbow']] = r_elbow
            joint_cmd[self.joints['waist']] = waist_turn
            
            sequence.append({'time': t, 'joints': joint_cmd.copy()})
        
        return sequence

if __name__ == "__main__":
    print("🤖 Robot Dance Ready!")
    dance = RobotDance()
    sequence = dance.generate_sequence(duration=30)
    print(f"Generated {len(sequence)} mechanical dance frames!")