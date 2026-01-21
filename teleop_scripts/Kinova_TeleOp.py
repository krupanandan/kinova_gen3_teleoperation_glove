import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient # <--- NEW
from control_msgs.action import GripperCommand # <--- NEW
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import serial
import ikpy.chain
import tempfile
import sys
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 1000000
URDF_PATH = "/colcon_ws/gen3.urdf"

# --- TUNING ---
CONTROL_HZ = 30.0         
VEL_SCALE_XY = 0.003      
VEL_SCALE_Z = 0.005       
ROT_SCALE = 0.04          
TILT_DEADZONE = 5.0       
TILT_MAX = 45.0           

HOME_JOINTS = [0.0, -0.6, 0.0, 2.5, 0.0, -1.0, 0.0]

class SerialReader(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        # Data: Pitch, Roll, Enc, BtnHome, Mode, IR
        self.data = {'p': 0.0, 'r': 0.0, 'enc': 0, 'btn': 0, 'mode': 0, 'ir': 0}
        self.running = True
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.ser.flushInput()
            print(f"Serial Connected on {SERIAL_PORT}")
        except Exception as e:
            print(f"Serial Error: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("S,") and line.endswith(",E"):  #typical formatting for data from sensor
                        parts = line.split(",")
                        if len(parts) == 8: 
                            self.data = {
                                'p': float(parts[1]), #pitch
                                'r': float(parts[2]), #roll
                                'enc': int(parts[3]), #encoder
                                'btn': int(parts[4]), #button
                                'mode': int(parts[5]), #mode
                                'ir': int(parts[6]) #ir sensor
                            }
            except Exception:
                pass
            time.sleep(0.005)  #Loops every 5ms to reduce lag

class KinovaTeleop(Node):
    def __init__(self):
        super().__init__('kinova_teleop')

        # 1. Setup Robot Model
        try:
            with open(URDF_PATH, 'r') as f:
                urdf_content = f.read().replace('type="continuous"', 'type="revolute"') #revolute joints for joints with limits if not specified by default
            self.temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False)
            self.temp_urdf.write(urdf_content)
            self.temp_urdf.close()
            self.chain = ikpy.chain.Chain.from_urdf_file(self.temp_urdf.name)
            
            n_links = len(self.chain.links)
            self.chain.active_links_mask = [False] + [True] * 7 + [False] * (n_links - 8) #indicates base and gripper are fixed and the 7 joints in between are free to move
            print("IK Chain Loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            sys.exit(1)

        # 2. Publishers (Arm Only)
        self.arm_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # --- NEW GRIPPER ACTION CLIENT ---
        self._gripper_action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        # ---------------------------------

        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # 3. Serial - beginning the thread to read serial data
        self.serial_thread = SerialReader()
        if self.serial_thread.running:
            self.serial_thread.start()
        else:
            sys.exit(1)

        # 4. State Variables - initialised so that the robot doesn't move to a random location such as (0,0,0) at max speed on startup
        self.current_joints = None      
        self.target_joints = None       
        self.target_cartesian = None       
        self.target_rotation = None        
        
        self.last_enc_val = 0
        self.last_btn_val = 0
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        
        # Gripper & IR State
        self.gripper_closed = False 
        self.last_ir_val = 0

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        print("System Ready.\n   - D3: Mode Switch\n   - D4: IR Gripper Toggle\n   - Encoder Push: Home")

    def joint_cb(self, msg):
        temp_joints = []
        for i in range(1, 8):
            name = f"joint_{i}"
            if name in msg.name:
                idx = msg.name.index(name)
                temp_joints.append(msg.position[idx])
        
        if len(temp_joints) == 7: #checking if all joints are received
            self.current_joints = temp_joints
            if self.target_joints is None:
                self.target_joints = list(temp_joints) #setting initial target as current position
                self.update_fk_from_joints(self.target_joints) #calculating 3D cartesian position and orientation

    def update_fk_from_joints(self, joints): #checking where the end effector is in 3D space based on joint angles
        ik_seed = [0.0] * len(self.chain.links)
        for i, val in enumerate(joints):
            ik_seed[i+1] = val
        fk_res = self.chain.forward_kinematics(ik_seed) #calculating forward kinematics
        self.target_cartesian = fk_res[:3, 3] #getting position
        self.target_rotation = fk_res[:3, :3] #getting orientation

    def get_input_intensity(self, val): #converts raw tilt input to normalized intensity
        if abs(val) < TILT_DEADZONE: return 0.0
        norm = (abs(val) - TILT_DEADZONE) / (TILT_MAX - TILT_DEADZONE)
        norm = min(max(norm, 0.0), 1.0)
        direction = 1.0 if val > 0 else -1.0
        return direction * norm

    def control_loop(self):
        if self.current_joints is None or self.target_cartesian is None:
            return

        raw = self.serial_thread.data #getting latest data from serial thread

        # --- DEBUG PRINT ---
        mode_str = "ROT" if raw['mode'] == 1 else "XYZ"
        grip_str = "CLOSED" if self.gripper_closed else "OPEN"
        sys.stdout.write(f"\r[{mode_str}] Grip: {grip_str} | IR: {raw['ir']} | P: {raw['p']:.1f} R: {raw['r']:.1f}   ")
        sys.stdout.flush()

        # --- 1. HANDLE HOMING ---
        if raw['btn'] == 1 and self.last_btn_val == 0: #homing (return robot to home state) is performed on button press 
            print("\nHOMING...")
            self.target_joints = list(HOME_JOINTS)
            self.update_fk_from_joints(self.target_joints)
            self.pitch_offset = raw['p']
            self.roll_offset = raw['r']
            self.publish_trajectory(self.target_joints, duration=3) #gives robot 3 seconds to reach home position
            self.last_enc_val = raw['enc']
            self.last_btn_val = raw['btn']
            return
        self.last_btn_val = raw['btn']

        # --- 2. HANDLE GRIPPER (IR TRIGGER) ---
        if raw['ir'] == 1 and self.last_ir_val == 0:
            self.gripper_closed = not self.gripper_closed
            self.publish_gripper_action() # <--- Updated call
            print(f"\n⚡ Gripper Toggled: {'CLOSED' if self.gripper_closed else 'OPEN'}")
        
        self.last_ir_val = raw['ir']

        # --- 3. CALCULATE INPUTS ---
        rel_pitch = raw['p'] - self.pitch_offset
        rel_roll  = raw['r'] - self.roll_offset
        
        input_pitch = self.get_input_intensity(rel_pitch) 
        input_roll = self.get_input_intensity(rel_roll)   
        
        enc_delta = raw['enc'] - self.last_enc_val
        self.last_enc_val = raw['enc']

        is_moving = False

        # --- 4. SWITCH MODES ---
        if raw['mode'] == 0:
            v_x = input_pitch * VEL_SCALE_XY
            v_y = input_roll * VEL_SCALE_XY
            v_z = enc_delta * VEL_SCALE_Z

            if abs(v_x) > 0 or abs(v_y) > 0 or abs(v_z) > 0:
                self.target_cartesian[0] += v_x
                self.target_cartesian[1] += v_y
                self.target_cartesian[2] += v_z
                self.target_cartesian[2] = max(0.05, min(self.target_cartesian[2], 1.0)) #creating a safety limit for Z height so that the arm doesn't crash into the table/floor
                is_moving = True
        else:
            d_pitch = input_pitch * ROT_SCALE
            d_roll  = input_roll * ROT_SCALE
            d_yaw   = enc_delta * ROT_SCALE * 0.5 

            if abs(d_pitch) > 0 or abs(d_roll) > 0 or abs(d_yaw) > 0:
                r_delta = R.from_euler('xyz', [d_roll, d_pitch, d_yaw], degrees=False)
                m_delta = r_delta.as_matrix()
                self.target_rotation = self.target_rotation @ m_delta
                is_moving = True

        # --- 5. SOLVE IK ---
        if is_moving:
            ik_seed = [0.0] * len(self.chain.links)
            for i, val in enumerate(self.target_joints):
                ik_seed[i+1] = val

            new_joints_full = self.chain.inverse_kinematics(
                target_position=self.target_cartesian,
                target_orientation=self.target_rotation, 
                orientation_mode="all", #match both position and orientation of the gripper to what it was before
                initial_position=ik_seed
            )

            self.target_joints = list(new_joints_full[1:8])
            self.publish_trajectory(self.target_joints, duration=0.08) #slightly less than control loop period to ensure smooth movement

    def publish_trajectory(self, joints, duration): #publishing joint trajectory to ROS2 topic
        traj = JointTrajectory()
        traj.joint_names = [f"joint_{i}" for i in range(1, 8)]
        p = JointTrajectoryPoint()
        p.positions = joints
        p.time_from_start.nanosec = int(duration * 1e9) 
        traj.points = [p]
        self.arm_pub.publish(traj)

    # --- NEW FUNCTION FOR GRIPPER ACTION ---
    def publish_gripper_action(self): #publishing command to open and close gripper
        goal_msg = GripperCommand.Goal()
        # 0.8 is Close, 0.0 is Open
        goal_msg.command.position = 0.8 if self.gripper_closed else 0.0
        goal_msg.command.max_effort = 100.0
        
        # Fire and forget (Async)
        self._gripper_action_client.send_goal_async(goal_msg)
    # ---------------------------------------

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(KinovaTeleop())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
