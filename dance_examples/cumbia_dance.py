#!/usr/bin/env python3
"""
Unitree G1 Cumbia Dance Sequence
Simulates traditional cumbia dance movements with hip sway and arm coordination
"""

import numpy as np
import time
import math

class CumbiaDance:
    def __init__(self):
        self.bpm = 120  # Typical cumbia tempo
        self.beat_duration = 60.0 / self.bpm
        
        # Joint names for G1 robot
        self.joints = {
            'left_hip_yaw': 0,
            'left_hip_roll': 1,
            'left_hip_pitch': 2,
            'right_hip_yaw': 3,
            'right_hip_roll': 4,
            'right_hip_pitch': 5,
            'left_shoulder_pitch': 6,
            'left_shoulder_roll': 7,
            'left_elbow': 8,
            'right_shoulder_pitch': 9,
            'right_shoulder_roll': 10,
            'right_elbow': 11,
            'waist': 12
        }
    
    def hip_sway_pattern(self, t, amplitude=0.8):
        """Classic cumbia hip sway - figure-8 motion"""
        hip_roll = amplitude * math.sin(2 * math.pi * t / (2 * self.beat_duration))
        hip_yaw = amplitude * 0.5 * math.sin(2 * math.pi * t / self.beat_duration)
        hip_pitch = 0.3 * math.sin(2 * math.pi * t / (3 * self.beat_duration))
        return hip_roll, hip_yaw, hip_pitch
    
    def arm_wave_pattern(self, t, side='left'):
        """Flowing arm movements characteristic of cumbia"""
        phase = 0 if side == 'left' else math.pi/2
        shoulder_pitch = 1.0 * math.sin(2 * math.pi * t / (4 * self.beat_duration) + phase)
        shoulder_roll = 0.8 * math.cos(2 * math.pi * t / (3 * self.beat_duration) + phase)
        elbow = -1.2 + 0.8 * math.sin(2 * math.pi * t / (4 * self.beat_duration) + phase + math.pi/4)
        return shoulder_pitch, shoulder_roll, elbow
    
    def generate_sequence(self, duration=30):
        """Generate complete cumbia dance sequence"""
        sequence = []
        dt = 0.02  # 50Hz control frequency
        
        for i in range(int(duration / dt)):
            t = i * dt
            hip_roll, hip_yaw, hip_pitch = self.hip_sway_pattern(t)
            l_shoulder_pitch, l_shoulder_roll, l_elbow = self.arm_wave_pattern(t, 'left')
            r_shoulder_pitch, r_shoulder_roll, r_elbow = self.arm_wave_pattern(t, 'right')
            waist_rotation = 0.6 * math.sin(2 * math.pi * t / (4 * self.beat_duration))
            
            joint_cmd = np.zeros(23)  # Full 23DOF
            # Add stepping pattern
            step_phase = math.sin(2 * math.pi * t / self.beat_duration)
            left_step = 0.4 * max(0, step_phase)  # Lift left foot
            right_step = 0.4 * max(0, -step_phase)  # Lift right foot
            
            # Hips with stepping
            joint_cmd[1] = hip_yaw      # left_hip_yaw
            joint_cmd[2] = hip_roll     # left_hip_roll  
            joint_cmd[3] = hip_pitch + left_step    # left_hip_pitch
            joint_cmd[4] = -left_step * 0.8         # left_knee
            joint_cmd[5] = left_step * 0.4          # left_ankle_pitch
            joint_cmd[7] = -hip_yaw     # right_hip_yaw
            joint_cmd[8] = -hip_roll    # right_hip_roll
            joint_cmd[9] = hip_pitch + right_step   # right_hip_pitch
            joint_cmd[10] = -right_step * 0.8       # right_knee
            joint_cmd[11] = right_step * 0.4        # right_ankle_pitch
            # Arms
            joint_cmd[13] = l_shoulder_pitch  # left_shoulder_pitch
            joint_cmd[14] = l_shoulder_roll   # left_shoulder_roll
            joint_cmd[16] = l_elbow           # left_elbow
            joint_cmd[18] = r_shoulder_pitch  # right_shoulder_pitch
            joint_cmd[19] = r_shoulder_roll   # right_shoulder_roll
            joint_cmd[21] = r_elbow           # right_elbow
            # Waist
            joint_cmd[0] = waist_rotation     # waist_yaw
            
            sequence.append({'time': t, 'joints': joint_cmd.tolist()})
        
        return sequence

if __name__ == "__main__":
    print("🎵 Cumbia Dance Ready!")
    dance = CumbiaDance()
    sequence = dance.generate_sequence(duration=10)
    print(f"Generated {len(sequence)} dance frames")