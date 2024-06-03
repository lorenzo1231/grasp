#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from sensor_msgs.msg import Joy, JointState
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
import termios
import tty
import select
import time
import math

# Movement step sizes
short_step = 0.05  # Increased step size for single presses
long_step = 0.1  # Increased step size for holding the button
grasp_short_step = math.radians(5)
grasp_long_step = math.radians(10)

# Movement bindings
move_bindings = {
    (1, 0): (0, 0, short_step),  # UP (D-pad up)
    (-1, 0): (0, 0, -short_step),  # DOWN (D-pad down)
    (0, 1): (0, short_step, 0),  # LEFT (D-pad left)
    (0, -1): (0, -short_step, 0),  # RIGHT (D-pad right)
    (0, 3): (short_step, 0, 0),  # FORWARD (button 3, Square button)
    (0, 0): (-short_step, 0, 0),  # BACKWARD (button 0, X button)
}

rotation_bindings = {
    'roll_up': (0.1, 0, 0),  # R1 for roll up
    'roll_down': (-0.1, 0, 0),  # L1 for roll down
    'pitch_up': (0, 0.1, 0),  # R2 for pitch up
    'pitch_down': (0, -0.1, 0),  # L2 for pitch down
}

all_finger_joints = [
    'R_index_proximal_joint', 'R_index_intermediate_joint',
    'R_middle_proximal_joint', 'R_middle_intermediate_joint',
    'R_ring_proximal_joint', 'R_ring_intermediate_joint',
    'R_pinky_proximal_joint', 'R_pinky_intermediate_joint',
    'R_thumb_proximal_pitch_joint', 'R_thumb_intermediate_joint', 'R_thumb_distal_joint'
]

thumb_joints = [
    'R_thumb_proximal_pitch_joint', 'R_thumb_intermediate_joint', 'R_thumb_distal_joint'
]

# Store current effort values
current_efforts = {}

def joint_states_callback(msg):
    global current_efforts
    for i, name in enumerate(msg.name):
        current_efforts[name] = msg.effort[i]

class PS4Teleop:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('ps4_teleop')

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("l_r_t")
        self.finger_groups = {
            'finger_1': MoveGroupCommander("finger_1"),
            'finger_2': MoveGroupCommander("finger_2"),
            'finger_3': MoveGroupCommander("finger_3"),
            'finger_4': MoveGroupCommander("finger_4"),
            'finger_5': MoveGroupCommander("finger_5")
        }
        self.all_fingers_group = MoveGroupCommander("all_fingers")

        rospy.Subscriber('/joint_states', JointState, joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.direction = None
        self.rotation = None
        self.joint_8_delta = 0
        self.joint_9_delta = 0
        self.joint_11_delta = 0
        self.thumb_yaw_delta = 0
        self.grasp = False
        self.hold_start_time = None
        self.hold_duration_threshold = 0.5  # Duration to switch from short to long step size

        self.print_controls()

        rospy.loginfo("PS4 Teleop Node Initialized")
        self.main_loop()

    def print_controls(self):
        rospy.loginfo("""
        Control Your Robot!
        ---------------------------
        Movement:
            D-pad up: Move up
            D-pad down: Move down
            D-pad left: Move left
            D-pad right: Move right
            Square button: Move forward
            X button: Move backward

        Rotation:
            R1: Roll up
            L1: Roll down
            R2: Pitch up
            L2: Pitch down

        Joint Control:
            Left joystick vertical axis: Control J_8
            Right joystick horizontal axis: Control J_9
            Share button: J_11 down
            Option button: J_11 up
            Triangle button: Thumb yaw

        Grasp:
            O button: Grasp
        """)

    def joy_callback(self, data):
        axes = data.axes
        buttons = data.buttons
        
        # Get movement direction based on axis values
        if int(axes[7]) != 0:  # D-pad up/down
            self.direction = (int(axes[7]), 0)
        elif int(axes[6]) != 0:  # D-pad left/right
            self.direction = (0, int(axes[6]))
        elif buttons[3]:  # Square button for FORWARD
            self.direction = (0, 3)
        elif buttons[0]:  # X button for BACKWARD
            self.direction = (0, 0)
        else:
            self.direction = None
        
        # Get rotation based on button values
        if buttons[5]:  # R1 for roll up
            self.rotation = 'roll_up'
        elif buttons[4]:  # L1 for roll down
            self.rotation = 'roll_down'
        elif buttons[7]:  # R2 for pitch up
            self.rotation = 'pitch_up'
        elif buttons[6]:  # L2 for pitch down
            self.rotation = 'pitch_down'
        else:
            self.rotation = None

        # Get joint control for J_8 based on left joystick vertical axis (flipped) with increased sensitivity
        self.joint_8_delta = -axes[1] * 0.1  # Adjust the factor for desired sensitivity

        # Get joint control for J_9 based on right joystick horizontal axis (flipped) with increased sensitivity
        self.joint_9_delta = -axes[3] * 0.1  # Adjust the factor for desired sensitivity

        # Get joint control for J_11 based on Share (button 8) and Option (button 9) buttons
        if buttons[8]:  # Share button
            self.joint_11_delta = -0.1
        elif buttons[9]:  # Option button
            self.joint_11_delta = 0.1
        else:
            self.joint_11_delta = 0

        # Get joint control for R_thumb_proximal_yaw_joint based on Triangle button
        self.thumb_yaw_delta = 0.1 if buttons[2] else 0  # Triangle button for thumb yaw joint

        # Grasp control using O button
        self.grasp = buttons[1]  # O button

    def main_loop(self):
        rate = rospy.Rate(20)  # Increased frequency to 20 Hz for smoother movement
        while not rospy.is_shutdown():
            step_size = short_step  # Default step size

            if self.direction:
                if self.hold_start_time is None:
                    self.hold_start_time = time.time()
                elif time.time() - self.hold_start_time > self.hold_duration_threshold:
                    step_size = long_step  # Switch to long step size if holding

                move_delta = move_bindings.get(self.direction, (0, 0, 0))
                move_delta = (move_delta[0] * step_size, move_delta[1] * step_size, move_delta[2] * step_size)
                self.move_end_effector(self.arm_group, move_delta)
            else:
                self.hold_start_time = None  # Reset hold start time when direction is released

            if self.rotation and self.rotation in rotation_bindings:
                self.rotate_end_effector(self.arm_group, rotation_bindings[self.rotation])
            if abs(self.joint_8_delta) > 0.01:  # Threshold to prevent jittering
                self.move_joint(self.arm_group, 'J_8', self.joint_8_delta)
            if abs(self.joint_9_delta) > 0.01:  # Threshold to prevent jittering
                self.move_joint(self.arm_group, 'J_9', self.joint_9_delta)
            if abs(self.joint_11_delta) > 0.01:  # Threshold to prevent jittering
                self.move_joint(self.arm_group, 'J_11', self.joint_11_delta)
            if abs(self.thumb_yaw_delta) > 0:  # Move thumb yaw joint
                self.move_joint(self.finger_groups['finger_5'], 'R_thumb_proximal_yaw_joint', self.thumb_yaw_delta)
            if self.grasp:  # Execute grasp if O button is pressed
                self.grasp_fingers(self.all_fingers_group)
            rate.sleep()

    def move_end_effector(self, group, direction):
        pose = group.get_current_pose().pose
        target_pose = pose

        dx, dy, dz = direction
        target_pose.position.x += dx
        target_pose.position.y += dy
        target_pose.position.z += dz

        rospy.loginfo(f"Moving end effector to position: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")

        group.set_pose_target(target_pose)
        plan = group.plan()
        
        if plan[0]:
            success = group.execute(plan[1], wait=True)
            if not success:
                rospy.logwarn("Execution failed, retrying...")
        else:
            rospy.logerr("Failed to create a valid plan.")

    def rotate_end_effector(self, group, rotation):
        pose = group.get_current_pose().pose
        current_orientation = pose.orientation
        current_quat = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        current_euler = euler_from_quaternion(current_quat)

        roll, pitch, yaw = rotation
        new_euler = (current_euler[0] + roll, current_euler[1] + pitch, current_euler[2])

        new_quat = quaternion_from_euler(*new_euler)
        pose.orientation = Quaternion(*new_quat)

        rospy.loginfo(f"Rotating end effector to orientation: roll={new_euler[0]}, pitch={new_euler[1]}, yaw={new_euler[2]}")

        group.set_pose_target(pose)
        plan = group.plan()
        
        if plan[0]:
            success = group.execute(plan[1], wait=True)
            if not success:
                rospy.logwarn("Execution failed, retrying...")
        else:
            rospy.logerr("Failed to create a valid plan.")

    def move_joint(self, group, joint_name, delta):
        try:
            joint_positions = group.get_current_joint_values()
            joint_index = group.get_active_joints().index(joint_name)
            current_position = joint_positions[joint_index]
            new_position = current_position + delta

            rospy.loginfo(f"Moving {joint_name} from {current_position} to {new_position}")

            group.set_joint_value_target({joint_name: new_position})
            plan = group.plan()

            if plan[0]:
                success = group.execute(plan[1], wait=True)
                if not success:
                    rospy.logwarn(f"Execution failed for joint {joint_name}, retrying...")
            else:
                rospy.logerr(f"Failed to create a valid plan for joint {joint_name}.")
        except ValueError:
            rospy.logerr(f"Joint {joint_name} not found in group {group.get_name()}.")

    def grasp_fingers(self, finger_group):
        step_size = grasp_short_step  # Default step size for single press
        loop_delay = 0.1  # Delay for smoother movement

        if self.hold_start_time is None:
            self.hold_start_time = time.time()
        elif time.time() - self.hold_start_time > self.hold_duration_threshold:
            step_size = grasp_long_step  # Switch to long step size if holding

        current_positions = finger_group.get_current_joint_values()

        rospy.loginfo(f"Current finger positions: {current_positions}")

        for index, joint in enumerate(all_finger_joints):
            current_positions[index] += step_size

        rospy.loginfo(f"New finger positions: {current_positions}")

        finger_group.set_joint_value_target(current_positions)
        finger_group.go(wait=True)
        time.sleep(loop_delay)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    roscpp_initialize(sys.argv)
    rospy.init_node('ps4_teleop', argv=sys.argv)

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    arm_group = MoveGroupCommander("l_r_t")
    finger_groups = {
        'finger_1': MoveGroupCommander("finger_1"),
        'finger_2': MoveGroupCommander("finger_2"),
        'finger_3': MoveGroupCommander("finger_3"),
        'finger_4': MoveGroupCommander("finger_4"),
        'finger_5': MoveGroupCommander("finger_5")
    }
    all_fingers_group = MoveGroupCommander("all_fingers")

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

    try:
        PS4Teleop()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        roscpp_shutdown()

