#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointPositionController:
    def __init__(self):
        rospy.init_node('joint_position_controller', anonymous=True)
        
        # Parameters
        self.joint_name = rospy.get_param('~joint_name', 'J_4')
        self.kp = rospy.get_param('~kp', 50.0)
        self.ki = rospy.get_param('~ki', 0.01)
        self.kd = rospy.get_param('~kd', 5.0)
        self.damping = rospy.get_param('~damping', 2.0)
        
        self.effort_pub = rospy.Publisher('/effort_controller_' + self.joint_name + '/command', Float64, queue_size=10)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = rospy.Time.now()
        
    def joint_states_callback(self, msg):
        try:
            index = msg.name.index(self.joint_name)
            current_position = msg.position[index]
            current_velocity = msg.velocity[index]
            self.control_loop(current_position, current_velocity)
        except ValueError:
            rospy.logwarn("Joint %s not found in joint states", self.joint_name)
    
    def control_loop(self, current_position, current_velocity):
        desired_position = 0.0
        error = desired_position - current_position
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt == 0:
            return
        
        self.error_sum += error * dt
        d_error = (error - self.last_error) / dt
        
        effort = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * d_error) - (self.damping * current_velocity)
        
        self.effort_pub.publish(Float64(effort))
        
        self.last_error = error
        self.last_time = current_time
        
        rospy.loginfo("Effort: %s, P-Term: %s, I-Term: %s, D-Term: %s, Damping: %s",
                      effort, self.kp * error, self.ki * self.error_sum, self.kd * d_error, self.damping * current_velocity)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = JointPositionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

