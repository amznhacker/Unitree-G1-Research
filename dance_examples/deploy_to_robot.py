#!/usr/bin/env python3
"""
Deploy Dance to Real G1 Robot
"""

import sys
import time
from cumbia_dance import CumbiaDance
from salsa_dance import SalsaDance
from bachata_dance import BachataDance
from robot_dance import RobotDance

def deploy_dance(dance_name, duration=10):
    """Deploy dance to real G1 robot"""
    
    # Import Unitree SDK
    try:
        from unitree_sdk2py import Robot, RobotState
        print("🤖 Unitree SDK loaded")
    except ImportError:
        print("❌ Unitree SDK not found. Install unitree_sdk2py")
        print("💡 For now, showing what would be sent to robot...")
        simulate_deployment(dance_name, duration)
        return
    
    # Get dance class
    dances = {
        'cumbia': CumbiaDance,
        'salsa': SalsaDance, 
        'bachata': BachataDance,
        'robot': RobotDance
    }
    
    if dance_name not in dances:
        print(f"❌ Unknown dance: {dance_name}")
        print(f"Available: {list(dances.keys())}")
        return
    
    # Generate sequence
    dance = dances[dance_name]()
    sequence = dance.generate_sequence(duration=duration)
    print(f"🎭 Generated {dance_name} dance: {len(sequence)} frames")
    
    # Safety confirmation
    print("⚠️  SAFETY CHECK:")
    print(f"   - Dance: {dance_name}")
    print(f"   - Duration: {duration}s")
    print(f"   - Frames: {len(sequence)}")
    print("   - Emergency stop ready?")
    
    confirm = input("Deploy to real robot? (yes/no): ")
    if confirm.lower() != 'yes':
        print("❌ Deployment cancelled")
        return
    
    # Connect to robot
    try:
        robot = Robot()
        robot.Init()
        print("✅ Connected to G1 robot")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Deploy sequence
    print("🚀 Deploying dance...")
    start_time = time.time()
    
    for i, frame in enumerate(sequence):
        current_time = time.time() - start_time
        
        # Wait for correct timing
        while current_time < frame['time']:
            time.sleep(0.001)
            current_time = time.time() - start_time
        
        # Send joint commands
        try:
            robot.SendJointCommand(frame['joints'])
            if i % 50 == 0:  # Progress every second
                print(f"   Frame {i}/{len(sequence)} ({current_time:.1f}s)")
        except Exception as e:
            print(f"❌ Command failed at frame {i}: {e}")
            break
    
    print("✅ Dance deployment complete!")

def simulate_deployment(dance_name, duration):
    """Simulate what would be sent to robot"""
    
    dances = {
        'cumbia': CumbiaDance,
        'salsa': SalsaDance,
        'bachata': BachataDance, 
        'robot': RobotDance
    }
    
    dance = dances[dance_name]()
    sequence = dance.generate_sequence(duration=duration)
    
    print(f"🎭 {dance_name.title()} Dance Simulation")
    print(f"📊 Frames: {len(sequence)}")
    print(f"⏱️  Duration: {duration}s")
    print("📡 Joint commands that would be sent:")
    
    # Show sample frames
    for i in [0, len(sequence)//4, len(sequence)//2, -1]:
        frame = sequence[i]
        print(f"   t={frame['time']:.2f}s: joints={[f'{x:.2f}' for x in frame['joints'][:6]]}...")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 deploy_to_robot.py <dance_name> [duration]")
        print("Dances: cumbia, salsa, bachata, robot")
        return
    
    dance_name = sys.argv[1].lower()
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    
    deploy_dance(dance_name, duration)

if __name__ == "__main__":
    main()