#!/usr/bin/env python3
"""
G1 Dance Simulator - Run all dance sequences with MuJoCo visualization
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
from cumbia_dance import CumbiaDance
from salsa_dance import SalsaDance
from bachata_dance import BachataDance
from robot_dance import RobotDance

class G1DanceSimulator:
    def __init__(self, model_path=None):
        # For now, create a simple model - replace with actual G1 URDF
        self.model_xml = """
        <mujoco>
            <worldbody>
                <body name="torso" pos="0 0 1">
                    <geom type="box" size="0.2 0.1 0.3" rgba="0.8 0.2 0.2 1"/>
                    <joint name="waist" type="hinge" axis="0 0 1"/>
                    
                    <body name="left_arm" pos="-0.25 0 0.2">
                        <geom type="capsule" size="0.05 0.3" rgba="0.2 0.8 0.2 1"/>
                        <joint name="left_shoulder_pitch" type="hinge" axis="1 0 0"/>
                        <joint name="left_shoulder_roll" type="hinge" axis="0 1 0"/>
                        
                        <body name="left_forearm" pos="0 0 -0.3">
                            <geom type="capsule" size="0.04 0.25" rgba="0.2 0.8 0.2 1"/>
                            <joint name="left_elbow" type="hinge" axis="1 0 0"/>
                        </body>
                    </body>
                    
                    <body name="right_arm" pos="0.25 0 0.2">
                        <geom type="capsule" size="0.05 0.3" rgba="0.2 0.8 0.2 1"/>
                        <joint name="right_shoulder_pitch" type="hinge" axis="1 0 0"/>
                        <joint name="right_shoulder_roll" type="hinge" axis="0 1 0"/>
                        
                        <body name="right_forearm" pos="0 0 -0.3">
                            <geom type="capsule" size="0.04 0.25" rgba="0.2 0.8 0.2 1"/>
                            <joint name="right_elbow" type="hinge" axis="1 0 0"/>
                        </body>
                    </body>
                    
                    <body name="left_leg" pos="-0.1 0 -0.3">
                        <geom type="capsule" size="0.06 0.4" rgba="0.2 0.2 0.8 1"/>
                        <joint name="left_hip_pitch" type="hinge" axis="1 0 0"/>
                        <joint name="left_hip_roll" type="hinge" axis="0 1 0"/>
                        <joint name="left_hip_yaw" type="hinge" axis="0 0 1"/>
                    </body>
                    
                    <body name="right_leg" pos="0.1 0 -0.3">
                        <geom type="capsule" size="0.06 0.4" rgba="0.2 0.2 0.8 1"/>
                        <joint name="right_hip_pitch" type="hinge" axis="1 0 0"/>
                        <joint name="right_hip_roll" type="hinge" axis="0 1 0"/>
                        <joint name="right_hip_yaw" type="hinge" axis="0 0 1"/>
                    </body>
                </body>
            </worldbody>
            
            <actuator>
                <motor joint="waist"/>
                <motor joint="left_shoulder_pitch"/>
                <motor joint="left_shoulder_roll"/>
                <motor joint="left_elbow"/>
                <motor joint="right_shoulder_pitch"/>
                <motor joint="right_shoulder_roll"/>
                <motor joint="right_elbow"/>
                <motor joint="left_hip_pitch"/>
                <motor joint="left_hip_roll"/>
                <motor joint="left_hip_yaw"/>
                <motor joint="right_hip_pitch"/>
                <motor joint="right_hip_roll"/>
                <motor joint="right_hip_yaw"/>
            </actuator>
        </mujoco>
        """
        
        self.model = mujoco.MjModel.from_xml_string(self.model_xml)
        self.data = mujoco.MjData(self.model)
        
        # Joint mapping
        self.joint_names = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow',
            'waist'
        ]
    
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