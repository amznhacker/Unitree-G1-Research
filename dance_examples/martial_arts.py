#!/usr/bin/env python3
"""
Unitree G1 Martial Arts Kata
Tai Chi and Kung Fu inspired movements
"""

import numpy as np
import math

class MartialArts:
    def __init__(self):
        self.bpm = 60  # Slow, controlled movements
        self.beat_duration = 60.0 / self.bpm
    
    def tai_chi_flow(self, t):
        """Flowing Tai Chi movements"""
        wave = math.sin(math.pi * t / (4 * self.beat_duration))
        return wave
    
    def kung_fu_stance(self, t, stance_time):
        """Dynamic kung fu stances"""
        if int(t / stance_time) % 4 == 0:
            return "horse"
        elif int(t / stance_time) % 4 == 1:
            return "crane"
        elif int(t / stance_time) % 4 == 2:
            return "tiger"
        else:
            return "dragon"
    
    def generate_sequence(self, duration=40):
        sequence = []
        dt = 0.02
        
        for i in range(int(duration / dt)):
            t = i * dt
            flow = self.tai_chi_flow(t)
            stance = self.kung_fu_stance(t, 8)  # Change stance every 8 seconds
            
            joint_cmd = np.zeros(23)
            
            if stance == "horse":
                # Horse stance - wide, low
                joint_cmd[2] = -0.8   # left_hip_roll
                joint_cmd[3] = -1.0   # left_hip_pitch
                joint_cmd[4] = -1.2   # left_knee
                joint_cmd[8] = 0.8    # right_hip_roll
                joint_cmd[9] = -1.0   # right_hip_pitch
                joint_cmd[10] = -1.2  # right_knee
                joint_cmd[13] = 0.5 * flow  # left_shoulder_pitch
                joint_cmd[18] = -0.5 * flow # right_shoulder_pitch
                
            elif stance == "crane":
                # Crane stance - one leg up
                joint_cmd[3] = 1.2    # left_hip_pitch (leg up)
                joint_cmd[4] = -2.0   # left_knee
                joint_cmd[9] = -0.3   # right_hip_pitch
                joint_cmd[13] = 1.5   # left_shoulder_pitch (wing up)
                joint_cmd[18] = -0.5  # right_shoulder_pitch
                joint_cmd[14] = 1.0   # left_shoulder_roll
                
            elif stance == "tiger":
                # Tiger stance - crouched, ready to pounce
                joint_cmd[3] = -0.8   # left_hip_pitch
                joint_cmd[4] = -1.5   # left_knee
                joint_cmd[9] = -0.8   # right_hip_pitch
                joint_cmd[10] = -1.5  # right_knee
                joint_cmd[13] = -0.5  # left_shoulder_pitch (claws)
                joint_cmd[16] = -1.0  # left_elbow
                joint_cmd[18] = -0.5  # right_shoulder_pitch
                joint_cmd[21] = -1.0  # right_elbow
                
            else:  # dragon
                # Dragon stance - flowing, serpentine
                joint_cmd[0] = 0.8 * math.sin(2 * math.pi * t / self.beat_duration)  # waist_yaw
                joint_cmd[3] = -0.5 + 0.3 * flow  # left_hip_pitch
                joint_cmd[9] = -0.5 - 0.3 * flow  # right_hip_pitch
                joint_cmd[13] = 1.0 * math.sin(2 * math.pi * t / self.beat_duration + math.pi/4)
                joint_cmd[18] = 1.0 * math.sin(2 * math.pi * t / self.beat_duration - math.pi/4)
            
            # Add flowing arm movements for all stances
            joint_cmd[15] = 0.3 * math.sin(math.pi * t / (2 * self.beat_duration))  # left_shoulder_yaw
            joint_cmd[20] = -0.3 * math.sin(math.pi * t / (2 * self.beat_duration)) # right_shoulder_yaw
            
            sequence.append({'time': t, 'joints': joint_cmd.tolist()})
        
        return sequence

if __name__ == "__main__":
    print("🥋 Martial Arts Kata Ready!")
    dance = MartialArts()
    sequence = dance.generate_sequence(duration=30)
    print(f"Generated {len(sequence)} martial arts frames with stances!")