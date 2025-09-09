#!/usr/bin/env python3
"""
Unitree G1 Bachata Dance Sequence
Sensual Latin dance with hip movements and romantic arm gestures
"""

import numpy as np
import math

class BachataDance:
    def __init__(self):
        self.bpm = 130  # Moderate bachata tempo
        self.beat_duration = 60.0 / self.bpm
        
        self.joints = {
            'left_hip_yaw': 0, 'left_hip_roll': 1, 'left_hip_pitch': 2,
            'right_hip_yaw': 3, 'right_hip_roll': 4, 'right_hip_pitch': 5,
            'left_shoulder_pitch': 6, 'left_shoulder_roll': 7, 'left_elbow': 8,
            'right_shoulder_pitch': 9, 'right_shoulder_roll': 10, 'right_elbow': 11,
            'waist': 12
        }
    
    def hip_pop_pattern(self, t):
        """Characteristic bachata hip pop on beat 4"""
        beat_position = (t / self.beat_duration) % 4
        if 3.7 <= beat_position <= 4.0:  # Hip pop on beat 4
            return 0.5 * math.sin(20 * math.pi * (beat_position - 3.7))
        return 0.1 * math.sin(2 * math.pi * t / self.beat_duration)
    
    def romantic_arms(self, t, side='left'):
        """Smooth, flowing arm movements"""
        phase = 0 if side == 'left' else math.pi/4
        shoulder_pitch = 0.3 * math.sin(math.pi * t / (2 * self.beat_duration) + phase)
        shoulder_roll = 0.5 * math.cos(math.pi * t / (3 * self.beat_duration) + phase)
        elbow = 0.8 + 0.2 * math.sin(math.pi * t / (2 * self.beat_duration) + phase)
        return shoulder_pitch, shoulder_roll, elbow
    
    def side_step_pattern(self, t):
        """Side-to-side stepping pattern"""
        step_phase = 2 * math.pi * t / (4 * self.beat_duration)
        left_step = 0.15 * math.sin(step_phase)
        right_step = -0.15 * math.sin(step_phase)
        return left_step, right_step
    
    def generate_sequence(self, duration=30):
        sequence = []
        dt = 0.02
        
        for i in range(int(duration / dt)):
            t = i * dt
            hip_pop = self.hip_pop_pattern(t)
            left_step, right_step = self.side_step_pattern(t)
            l_shoulder_pitch, l_shoulder_roll, l_elbow = self.romantic_arms(t, 'left')
            r_shoulder_pitch, r_shoulder_roll, r_elbow = self.romantic_arms(t, 'right')
            
            # Subtle waist movement
            waist_sway = 0.15 * math.sin(2 * math.pi * t / (4 * self.beat_duration))
            
            joint_cmd = np.zeros(29)
            # Left leg
            joint_cmd[0] = left_step * 0.3      # left_hip_pitch
            joint_cmd[1] = hip_pop + left_step  # left_hip_roll
            # Right leg
            joint_cmd[6] = right_step * 0.3     # right_hip_pitch
            joint_cmd[7] = hip_pop + right_step # right_hip_roll
            # Waist
            joint_cmd[12] = waist_sway          # waist_yaw
            # Left arm
            joint_cmd[15] = l_shoulder_pitch    # left_shoulder_pitch
            joint_cmd[16] = l_shoulder_roll     # left_shoulder_roll
            joint_cmd[18] = l_elbow             # left_elbow
            # Right arm
            joint_cmd[22] = r_shoulder_pitch    # right_shoulder_pitch
            joint_cmd[23] = r_shoulder_roll     # right_shoulder_roll
            joint_cmd[25] = r_elbow             # right_elbow
            
            sequence.append({'time': t, 'joints': joint_cmd.copy()})
        
        return sequence

if __name__ == "__main__":
    print("💕 Bachata Dance Ready!")
    dance = BachataDance()
    sequence = dance.generate_sequence(duration=20)
    print(f"Generated {len(sequence)} bachata frames with hip pops!")