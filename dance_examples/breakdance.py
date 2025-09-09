#!/usr/bin/env python3
"""
Unitree G1 Breakdance Sequence
Hip-hop breakdancing with freezes and power moves
"""

import numpy as np
import math

class BreakDance:
    def __init__(self):
        self.bpm = 140
        self.beat_duration = 60.0 / self.bpm
    
    def toprock_pattern(self, t):
        """Standing breakdance moves"""
        bounce = 0.3 * math.sin(4 * math.pi * t / self.beat_duration)
        cross_step = 0.5 * math.sin(2 * math.pi * t / self.beat_duration)
        return bounce, cross_step
    
    def freeze_pose(self, t, freeze_start, freeze_duration):
        """Breakdance freeze positions"""
        if freeze_start <= t <= freeze_start + freeze_duration:
            return True
        return False
    
    def windmill_arms(self, t):
        """Windmill arm motion"""
        phase = 6 * math.pi * t / self.beat_duration
        left_arm = 1.5 * math.sin(phase)
        right_arm = 1.5 * math.sin(phase + math.pi)
        return left_arm, right_arm
    
    def generate_sequence(self, duration=30):
        sequence = []
        dt = 0.02
        
        for i in range(int(duration / dt)):
            t = i * dt
            bounce, cross_step = self.toprock_pattern(t)
            left_arm, right_arm = self.windmill_arms(t)
            
            # Power moves
            power_phase = math.sin(math.pi * t / 4)
            
            joint_cmd = np.zeros(23)
            
            # Freeze poses at specific times
            if self.freeze_pose(t, 8, 2):
                # Baby freeze
                joint_cmd[3] = -0.8   # left_hip_pitch
                joint_cmd[4] = -1.5   # left_knee
                joint_cmd[13] = -1.0  # left_shoulder_pitch
                joint_cmd[16] = -1.5  # left_elbow
            elif self.freeze_pose(t, 20, 2):
                # Chair freeze
                joint_cmd[3] = -1.2   # left_hip_pitch
                joint_cmd[9] = -1.2   # right_hip_pitch
                joint_cmd[13] = 1.5   # left_shoulder_pitch
                joint_cmd[18] = 1.5   # right_shoulder_pitch
            else:
                # Toprock and power moves
                joint_cmd[1] = cross_step * 0.5      # left_hip_yaw
                joint_cmd[2] = bounce                # left_hip_roll
                joint_cmd[3] = bounce * 0.5          # left_hip_pitch
                joint_cmd[7] = -cross_step * 0.5     # right_hip_yaw
                joint_cmd[8] = -bounce               # right_hip_roll
                joint_cmd[9] = bounce * 0.5          # right_hip_pitch
                
                # Dynamic arm movements
                joint_cmd[13] = left_arm             # left_shoulder_pitch
                joint_cmd[14] = 0.5 * math.cos(4 * math.pi * t / self.beat_duration)
                joint_cmd[16] = -0.8 + 0.5 * abs(left_arm)  # left_elbow
                joint_cmd[18] = right_arm            # right_shoulder_pitch
                joint_cmd[19] = -0.5 * math.cos(4 * math.pi * t / self.beat_duration)
                joint_cmd[21] = -0.8 + 0.5 * abs(right_arm) # right_elbow
                
                # Waist movement
                joint_cmd[0] = 0.8 * math.sin(2 * math.pi * t / self.beat_duration)
            
            sequence.append({'time': t, 'joints': joint_cmd.tolist()})
        
        return sequence

if __name__ == "__main__":
    print("🕺 Breakdance Ready!")
    dance = BreakDance()
    sequence = dance.generate_sequence(duration=20)
    print(f"Generated {len(sequence)} breakdance frames with freezes!")