#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
import termios
import tty
import time
import math
import select

# Movement step sizes
short_step = 0.05
long_step = 0.1
grasp_short_step = math.radians(2)  # smaller step size for better control
grasp_long_step = math.radians(5)

# Movement bindings
move_bindings = {
    'i': (short_step, 0, 0),  # FORWARD
    'k': (-short_step, 0, 0),  # BACKWARD
    'a': (0, short_step, 0),  # LEFT
    'd': (0, -short_step, 0),  # RIGHT
    'w': (0, 0, short_step),  # UP
    's': (0, 0, -short_step),  # DOWN
}

rotation_bindings = {
    'q': (0.1, 0, 0),  # Roll up
    'Q': (-0.1, 0, 0),  # Roll down
    'r': (0, 0.1, 0),  # Pitch up
    'R': (0, -0.1, 0),  # Pitch down
}

# Individual joint control bindings for the arm
arm_joint_bindings = {
    'j': ('J_4', 0.1),
    'J': ('J_4', -0.1),
    'k': ('J_5', 0.1),
    'K': ('J_5', -0.1),
    'l': ('J_6', 0.1),
    'L': ('J_6', -0.1),
    'm': ('J_8', 0.1),
    'M': ('J_8', -0.1),
    'n': ('J_9', 0.1),
    'N': ('J_9', -0.1),
    'b': ('J_10', 0.1),
    'B': ('J_10', -0.1),
    'v': ('J_11', 0.1),
    'V': ('J_11', -0.1),
}

# Individual joint control bindings for the fingers
finger_joint_bindings = {
    't': ('R_thumb_proximal_yaw_joint', 0.05),  # smaller step size for better control
    'T': ('R_thumb_proximal_yaw_joint', -0.05),  # smaller step size for better control
}

all_finger_joints = [
    'R_index_proximal_joint', 'R_index_intermediate_joint',
    'R_middle_proximal_joint', 'R_middle_intermediate_joint',
    'R_ring_proximal_joint', 'R_ring_intermediate_joint',
    'R_pinky_proximal_joint', 'R_pinky_intermediate_joint',
    'R_thumb_proximal_pitch_joint', 'R_thumb_intermediate_joint', 'R_thumb_distal_joint'
]

# Store current effort values
current_efforts = {}

def joint_states_callback(msg):
    global current_efforts
    for i, name in enumerate(msg.name):
        current_efforts[name] = msg.effort[i]

class KeyboardTeleop:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('keyboard_teleop')

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

        self.direction = None
        self.rotation = None
        self.joint_deltas = {}
        self.grasp = False
        self.hold_start_time = None
        self.hold_duration_threshold = 0.5

        self.print_key_bindings()

        rospy.loginfo("Keyboard Teleop Node Initialized")
        self.main_loop()

    def print_key_bindings(self):
        print("""
        Control Your Robot!
        ---------------------------
        Movement:
            i: Move forward
            k: Move backward
            a: Move left
            d: Move right
            w: Move up
            s: Move down

        Rotation:
            q: Roll up
            Q: Roll down
            r: Pitch up
            R: Pitch down

        Joint Control:
            j: Joint 4 up
            J: Joint 4 down
            k: Joint 5 up
            K: Joint 5 down
            l: Joint 6 up
            L: Joint 6 down
            m: Joint 8 up
            M: Joint 8 down
            n: Joint 9 up
            N: Joint 9 down
            b: Joint 10 up
            B: Joint 10 down
            v: Joint 11 up
            V: Joint 11 down
            t: Thumb yaw up
            T: Thumb yaw down

        Grasp:
            g: Grasp
        """)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def main_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            key = self.get_key()

            step_size = short_step

            if key in move_bindings:
                self.direction = move_bindings[key]
                rospy.loginfo(f"Moving with key {key}: direction {self.direction}")
            else:
                self.direction = None

            if key in rotation_bindings:
                self.rotation = rotation_bindings[key]
                rospy.loginfo(f"Rotating with key {key}: rotation {self.rotation}")
            else:
                self.rotation = None

            if key in arm_joint_bindings:
                joint_name, delta = arm_joint_bindings[key]
                rospy.loginfo(f"Moving arm joint {joint_name} with key {key}: delta {delta}")
                self.move_joint_individual(self.arm_group, joint_name, delta)
            elif key in finger_joint_bindings:
                joint_name, delta = finger_joint_bindings[key]
                rospy.loginfo(f"Moving finger joint {joint_name} with key {key}: delta {delta}")
                self.move_joint_individual(self.all_fingers_group, joint_name, delta)
            else:
                self.joint_deltas = {}

            if key == 'g':
                self.grasp = True
                rospy.loginfo("Grasping with key g")
            else:
                self.grasp = False

            if self.direction:
                if self.hold_start_time is None:
                    self.hold_start_time = time.time()
                elif time.time() - self.hold_start_time > self.hold_duration_threshold:
                    step_size = long_step

                move_delta = self.direction
                move_delta = (move_delta[0] * step_size, move_delta[1] * step_size, move_delta[2] * step_size)
                self.move_end_effector(self.arm_group, move_delta)
            else:
                self.hold_start_time = None

            if self.rotation:
                self.rotate_end_effector(self.arm_group, self.rotation)

            if self.grasp:
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

    def move_joint_individual(self, group, joint_name, delta):
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
        step_size = grasp_short_step
        loop_delay = 0.1

        if self.hold_start_time is None:
            self.hold_start_time = time.time()
        elif time.time() - self.hold_start_time > self.hold_duration_threshold:
            step_size = grasp_long_step

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
    rospy.init_node('keyboard_teleop', argv=sys.argv)

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
        KeyboardTeleop()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        roscpp_shutdown()

