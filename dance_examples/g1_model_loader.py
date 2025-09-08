#!/usr/bin/env python3
import os
import mujoco

class G1ModelLoader:
    def load_g1_model(self):
        # Try G1 URDF
        urdf_path = os.path.expanduser("~/g1_workspace/unitree_ros/robots/g1_description/urdf/g1.urdf")
        if os.path.exists(urdf_path):
            try:
                print(f"🤖 Loading G1 model")
                return mujoco.MjModel.from_xml_path(urdf_path)
            except:
                pass
        
        # Fallback model
        print("📦 Using fallback model")
        return self._create_g1_fallback()
    
    def _create_g1_fallback(self):
        model_xml = """
        <mujoco>
            <worldbody>
                <geom type="plane" size="10 10 0.1" rgba=".9 .9 .9 1"/>
                <body name="pelvis" pos="0 0 0.93">
                    <freejoint/>
                    <geom type="box" size="0.15 0.1 0.08" rgba="0.2 0.2 0.2 1"/>
                    
                    <!-- Waist -->
                    <body name="waist" pos="0 0 0.05">
                        <joint name="waist_yaw" type="hinge" axis="0 0 1"/>
                        
                        <!-- Left Leg -->
                        <body name="left_hip" pos="0 0.1 -0.1">
                            <joint name="left_hip_yaw" type="hinge" axis="0 0 1"/>
                            <joint name="left_hip_roll" type="hinge" axis="1 0 0"/>
                            <joint name="left_hip_pitch" type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.04 0.2" rgba="0.8 0.2 0.2 1"/>
                            <body name="left_knee" pos="0 0 -0.4">
                                <joint name="left_knee" type="hinge" axis="0 1 0"/>
                                <geom type="capsule" size="0.035 0.15" rgba="0.6 0.2 0.2 1"/>
                                <body name="left_ankle" pos="0 0 -0.3">
                                    <joint name="left_ankle_pitch" type="hinge" axis="0 1 0"/>
                                    <joint name="left_ankle_roll" type="hinge" axis="1 0 0"/>
                                    <geom type="box" size="0.08 0.04 0.02" rgba="0.2 0.2 0.2 1"/>
                                </body>
                            </body>
                        </body>
                        
                        <!-- Right Leg -->
                        <body name="right_hip" pos="0 -0.1 -0.1">
                            <joint name="right_hip_yaw" type="hinge" axis="0 0 1"/>
                            <joint name="right_hip_roll" type="hinge" axis="1 0 0"/>
                            <joint name="right_hip_pitch" type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.04 0.2" rgba="0.8 0.2 0.2 1"/>
                            <body name="right_knee" pos="0 0 -0.4">
                                <joint name="right_knee" type="hinge" axis="0 1 0"/>
                                <geom type="capsule" size="0.035 0.15" rgba="0.6 0.2 0.2 1"/>
                                <body name="right_ankle" pos="0 0 -0.3">
                                    <joint name="right_ankle_pitch" type="hinge" axis="0 1 0"/>
                                    <joint name="right_ankle_roll" type="hinge" axis="1 0 0"/>
                                    <geom type="box" size="0.08 0.04 0.02" rgba="0.2 0.2 0.2 1"/>
                                </body>
                            </body>
                        </body>
                        
                        <!-- Torso -->
                        <body name="torso" pos="0 0 0.1">
                            <geom type="box" size="0.12 0.08 0.2" rgba="0.2 0.6 0.2 1"/>
                            
                            <!-- Left Arm -->
                            <body name="left_shoulder" pos="0 0.15 0.15">
                                <joint name="left_shoulder_pitch" type="hinge" axis="0 1 0"/>
                                <joint name="left_shoulder_roll" type="hinge" axis="1 0 0"/>
                                <joint name="left_shoulder_yaw" type="hinge" axis="0 0 1"/>
                                <geom type="capsule" size="0.03 0.12" rgba="0.2 0.8 0.2 1"/>
                                <body name="left_elbow" pos="0 0 -0.24">
                                    <joint name="left_elbow" type="hinge" axis="0 1 0"/>
                                    <geom type="capsule" size="0.025 0.1" rgba="0.2 0.6 0.2 1"/>
                                    <body name="left_wrist" pos="0 0 -0.2">
                                        <joint name="left_wrist_roll" type="hinge" axis="0 0 1"/>
                                        <geom type="box" size="0.03 0.02 0.05" rgba="0.2 0.4 0.2 1"/>
                                    </body>
                                </body>
                            </body>
                            
                            <!-- Right Arm -->
                            <body name="right_shoulder" pos="0 -0.15 0.15">
                                <joint name="right_shoulder_pitch" type="hinge" axis="0 1 0"/>
                                <joint name="right_shoulder_roll" type="hinge" axis="1 0 0"/>
                                <joint name="right_shoulder_yaw" type="hinge" axis="0 0 1"/>
                                <geom type="capsule" size="0.03 0.12" rgba="0.2 0.8 0.2 1"/>
                                <body name="right_elbow" pos="0 0 -0.24">
                                    <joint name="right_elbow" type="hinge" axis="0 1 0"/>
                                    <geom type="capsule" size="0.025 0.1" rgba="0.2 0.6 0.2 1"/>
                                    <body name="right_wrist" pos="0 0 -0.2">
                                        <joint name="right_wrist_roll" type="hinge" axis="0 0 1"/>
                                        <geom type="box" size="0.03 0.02 0.05" rgba="0.2 0.4 0.2 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </worldbody>
            
            <actuator>
                <motor joint="waist_yaw"/>
                <motor joint="left_hip_yaw"/>
                <motor joint="left_hip_roll"/>
                <motor joint="left_hip_pitch"/>
                <motor joint="left_knee"/>
                <motor joint="left_ankle_pitch"/>
                <motor joint="left_ankle_roll"/>
                <motor joint="right_hip_yaw"/>
                <motor joint="right_hip_roll"/>
                <motor joint="right_hip_pitch"/>
                <motor joint="right_knee"/>
                <motor joint="right_ankle_pitch"/>
                <motor joint="right_ankle_roll"/>
                <motor joint="left_shoulder_pitch"/>
                <motor joint="left_shoulder_roll"/>
                <motor joint="left_shoulder_yaw"/>
                <motor joint="left_elbow"/>
                <motor joint="left_wrist_roll"/>
                <motor joint="right_shoulder_pitch"/>
                <motor joint="right_shoulder_roll"/>
                <motor joint="right_shoulder_yaw"/>
                <motor joint="right_elbow"/>
                <motor joint="right_wrist_roll"/>
            </actuator>
        </mujoco>
        """
        return mujoco.MjModel.from_xml_string(model_xml)
    
    def get_joint_names(self):
        """Return G1 23DOF joint names in order"""
        return [
            # Waist (1 DOF)
            'waist_yaw',
            # Left Leg (6 DOF)
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            # Right Leg (6 DOF)
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            # Left Arm (5 DOF)
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_roll',
            # Right Arm (5 DOF)
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow', 'right_wrist_roll'
        ]