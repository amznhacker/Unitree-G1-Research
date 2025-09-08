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
            start_time = time.time()
            frame_idx = 0
            
            while viewer.is_running() and frame_idx < len(sequence):
                current_time = time.time() - start_time
                
                # Find closest frame
                while (frame_idx < len(sequence) - 1 and 
                       sequence[frame_idx]['time'] < current_time):
                    frame_idx += 1
                
                # Apply joint commands
                frame = sequence[frame_idx]
                for i, joint_name in enumerate(self.joint_names):
                    if i < len(frame['joints']):
                        joint_id = mujoco.mj_name2id(
                            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
                        )
                        if joint_id >= 0:
                            self.data.ctrl[joint_id] = frame['joints'][i]
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                
                time.sleep(0.02)  # 50Hz
        
        print(f"✅ {dance_class.__name__} complete!")
    
    def dance_showcase(self):
        """Run all dances in sequence"""
        dances = [
            (CumbiaDance, 15, "🎵 Cumbia - Hip sway and flowing arms"),
            (SalsaDance, 12, "💃 Salsa - Fast steps with spins"),
            (BachataDance, 15, "💕 Bachata - Sensual hip pops"),
            (RobotDance, 20, "🤖 Robot - Mechanical precision")
        ]
        
        print("🎪 G1 Dance Showcase Starting!")
        print("=" * 50)
        
        for dance_class, duration, description in dances:
            print(f"\n{description}")
            input("Press Enter to start this dance...")
            self.run_dance(dance_class, duration)
            print("Dance complete! Next dance coming up...")
            time.sleep(2)
        
        print("\n🎉 Dance showcase complete!")

def main():
    try:
        simulator = G1DanceSimulator()
        
        print("G1 Dance Simulator")
        print("1. Cumbia Dance")
        print("2. Salsa Dance") 
        print("3. Bachata Dance")
        print("4. Robot Dance")
        print("5. Full Showcase")
        
        choice = input("Select dance (1-5): ")
        
        if choice == '1':
            simulator.run_dance(CumbiaDance, 20)
        elif choice == '2':
            simulator.run_dance(SalsaDance, 15)
        elif choice == '3':
            simulator.run_dance(BachataDance, 20)
        elif choice == '4':
            simulator.run_dance(RobotDance, 25)
        elif choice == '5':
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