#!/usr/bin/env python3
"""
Unitree G1 Salsa Dance Sequence
Fast-paced Latin dance with quick steps and spins
"""

import numpy as np
import math

class SalsaDance:
    def __init__(self):
        self.bpm = 180  # Fast salsa tempo
        self.beat_duration = 60.0 / self.bpm
        
        self.joints = {
            'left_hip_yaw': 0, 'left_hip_roll': 1, 'left_hip_pitch': 2,
            'right_hip_yaw': 3, 'right_hip_roll': 4, 'right_hip_pitch': 5,
            'left_shoulder_pitch': 6, 'left_shoulder_roll': 7, 'left_elbow': 8,
            'right_shoulder_pitch': 9, 'right_shoulder_roll': 10, 'right_elbow': 11,
            'waist': 12
        }
    
    def quick_step_pattern(self, t):
        """Fast salsa stepping pattern"""
        step_phase = 4 * math.pi * t / self.beat_duration  # Double speed
        left_step = 0.2 * math.sin(step_phase)
        right_step = 0.2 * math.sin(step_phase + math.pi)
        return left_step, right_step
    
    def arm_salsa_pattern(self, t, side='left'):
        """Sharp, precise arm movements"""
        phase = 0 if side == 'left' else math.pi/3
        shoulder_pitch = 0.6 * math.sin(4 * math.pi * t / self.beat_duration + phase)
        shoulder_roll = 0.4 * math.cos(6 * math.pi * t / self.beat_duration + phase)
        elbow = 1.2 + 0.3 * math.sin(4 * math.pi * t / self.beat_duration + phase)
        return shoulder_pitch, shoulder_roll, elbow
    
    def spin_sequence(self, t, spin_start=5, spin_duration=2):
        """Add dramatic spins"""
        if spin_start <= t <= spin_start + spin_duration:
            spin_progress = (t - spin_start) / spin_duration
            return 2 * math.pi * spin_progress
        return 0
    
    def generate_sequence(self, duration=25):
        sequence = []
        dt = 0.02
        
        for i in range(int(duration / dt)):
            t = i * dt
            left_step, right_step = self.quick_step_pattern(t)
            l_shoulder_pitch, l_shoulder_roll, l_elbow = self.arm_salsa_pattern(t, 'left')
            r_shoulder_pitch, r_shoulder_roll, r_elbow = self.arm_salsa_pattern(t, 'right')
            
            # Hip isolation
            hip_roll = 0.6 * math.sin(6 * math.pi * t / self.beat_duration)
            
            # Fast footwork
            quick_step = math.sin(8 * math.pi * t / self.beat_duration)
            left_knee = -0.8 * max(0, quick_step)
            right_knee = -0.8 * max(0, -quick_step)
            
            # Spins at specific times
            waist_spin = self.spin_sequence(t, 8, 1.5) + self.spin_sequence(t, 15, 1.5)
            
            joint_cmd = np.zeros(23)  # Full 23DOF
            # Legs with fast stepping
            joint_cmd[1] = 0.2 * quick_step     # left_hip_yaw
            joint_cmd[2] = hip_roll             # left_hip_roll
            joint_cmd[3] = left_step * 0.8      # left_hip_pitch
            joint_cmd[4] = left_knee            # left_knee
            joint_cmd[5] = left_step * 0.3      # left_ankle_pitch
            joint_cmd[7] = -0.2 * quick_step    # right_hip_yaw
            joint_cmd[8] = -hip_roll            # right_hip_roll
            joint_cmd[9] = right_step * 0.8     # right_hip_pitch
            joint_cmd[10] = right_knee          # right_knee
            joint_cmd[11] = right_step * 0.3    # right_ankle_pitch
            # Arms
            joint_cmd[13] = l_shoulder_pitch    # left_shoulder_pitch
            joint_cmd[14] = l_shoulder_roll     # left_shoulder_roll
            joint_cmd[16] = l_elbow             # left_elbow
            joint_cmd[18] = r_shoulder_pitch    # right_shoulder_pitch
            joint_cmd[19] = r_shoulder_roll     # right_shoulder_roll
            joint_cmd[21] = r_elbow             # right_elbow
            # Waist
            joint_cmd[0] = waist_spin           # waist_yaw
            
            sequence.append({'time': t, 'joints': joint_cmd.tolist()})
        
        return sequence

if __name__ == "__main__":
    print("💃 Salsa Dance Ready!")
    dance = SalsaDance()
    sequence = dance.generate_sequence(duration=15)
    print(f"Generated {len(sequence)} salsa frames with spins!")