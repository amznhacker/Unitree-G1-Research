#!/usr/bin/env python3
"""
G1 Dance Simulator - Run all dance sequences with MuJoCo visualization
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import os
from g1_model_loader import G1ModelLoader
from cumbia_dance import CumbiaDance
from salsa_dance import SalsaDance
from bachata_dance import BachataDance
from robot_dance import RobotDance
from breakdance import BreakDance
from martial_arts import MartialArts

class G1DanceSimulator:
    def __init__(self):
        loader = G1ModelLoader()
        self.model = loader.load_g1_model()
        self.data = mujoco.MjData(self.model)
        self.joint_names = loader.get_joint_names()
        print(f"✅ G1 ready with {len(self.joint_names)} DOF")
    

    
    def run_dance(self, dance_class, duration=20):
        """Run a specific dance in simulation"""
        print(f"🎭 Starting {dance_class.__name__} simulation...")
        
        dance = dance_class()
        sequence = dance.generate_sequence(duration=duration)
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Phase 1: Initialize to standing position (2 seconds)
            print("🤖 Initializing robot to standing position...")
            init_time = 2.0
            start_time = time.time()
            
            while time.time() - start_time < init_time and viewer.is_running():
                # Gradually move to neutral standing position
                progress = (time.time() - start_time) / init_time
                
                # Set all joints to neutral/standing position
                for i, joint_name in enumerate(self.joint_names):
                    joint_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
                    )
                    if joint_id >= 0:
                        # Smooth transition to standing pose
                        target = 0.0  # Neutral position
                        if 'knee' in joint_name:
                            target = -0.1  # Slight knee bend for stability
                        self.data.ctrl[joint_id] = target * progress
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.01)
            
            # Phase 2: Start dancing
            print(f"🎵 Starting {dance_class.__name__} performance!")
            dance_start_time = time.time()
            frame_idx = 0
            
            while viewer.is_running() and frame_idx < len(sequence):
                current_time = time.time() - dance_start_time
                
                # Find closest frame
                while (frame_idx < len(sequence) - 1 and 
                       sequence[frame_idx]['time'] < current_time):
                    frame_idx += 1
                
                # Apply joint commands (skip empty joints)
                frame = sequence[frame_idx]
                for i, joint_name in enumerate(self.joint_names):
                    if i < len(frame['joints']) and not joint_name.startswith('empty_'):
                        joint_id = mujoco.mj_name2id(
                            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
                        )
                        if joint_id >= 0:
                            self.data.ctrl[joint_id] = frame['joints'][i] * 1.5  # Reduced amplification
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.01)
        
        print(f"✅ {dance_class.__name__} complete!")
    
    def dance_showcase(self):
        """Run all dances in sequence"""
        dances = [
            (CumbiaDance, 15, "🎵 Cumbia - Hip sway with stepping"),
            (SalsaDance, 12, "💃 Salsa - Fast footwork with spins"),
            (BachataDance, 15, "💕 Bachata - Sensual hip pops"),
            (RobotDance, 20, "🤖 Robot - Mechanical precision"),
            (BreakDance, 18, "🕺 Breakdance - Hip-hop with freezes"),
            (MartialArts, 25, "🥋 Martial Arts - Kung Fu kata")
        ]
        
        print("🎪 G1 Performance Showcase Starting!")
        print("=" * 50)
        
        for i, (dance_class, duration, description) in enumerate(dances):
            print(f"\n{i+1}/{len(dances)}: {description}")
            input("Press Enter to start this performance...")
            self.run_dance(dance_class, duration)
            print("Performance complete!")
            if i < len(dances) - 1:
                print("Next performance coming up...")
                time.sleep(2)
        
        print("\n🎉 Performance showcase complete!")

def main():
    try:
        simulator = G1DanceSimulator()
        
        print("🤖 G1 Performance Simulator")
        print("1. 🎵 Cumbia Dance - Hip sway and steps")
        print("2. 💃 Salsa Dance - Fast steps with spins") 
        print("3. 💕 Bachata Dance - Romantic hip pops")
        print("4. 🤖 Robot Dance - Mechanical precision")
        print("5. 🕺 Breakdance - Hip-hop with freezes")
        print("6. 🥋 Martial Arts - Kung Fu kata")
        print("7. 🎪 Full Showcase - All performances")
        
        choice = input("Select performance (1-7): ")
        
        if choice == '1':
            simulator.run_dance(CumbiaDance, 20)
        elif choice == '2':
            simulator.run_dance(SalsaDance, 15)
        elif choice == '3':
            simulator.run_dance(BachataDance, 20)
        elif choice == '4':
            simulator.run_dance(RobotDance, 25)
        elif choice == '5':
            simulator.run_dance(BreakDance, 20)
        elif choice == '6':
            simulator.run_dance(MartialArts, 30)
        elif choice == '7':
            simulator.dance_showcase()
        else:
            print("Invalid choice")
            
    except ImportError:
        print("MuJoCo not installed. Running dance generation only...")
        # Fallback to just generating sequences
        dances = [CumbiaDance(), SalsaDance(), BachataDance(), RobotDance()]
        for dance in dances:
            sequence = dance.generate_sequence(10)
            print(f"{dance.__class__.__name__}: {len(sequence)} frames generated")

if __name__ == "__main__":
    main()