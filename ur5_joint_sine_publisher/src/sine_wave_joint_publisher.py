#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
import math

def sine_wave_joint_publisher():
    rospy.init_node('sine_wave_joint_publisher', anonymous=True)

    joint_pub = rospy.Publisher('/ur5/joint_states', JointState, queue_size=10)

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # Initialize
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.velocity = [0.0] * len(joint_names)
    joint_state.effort = [0.0] * len(joint_names)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():

        joint_state.position = [
            math.sin(rospy.get_time()),  # Shoulder pan joint
            0.5 * math.sin(2 * rospy.get_time()),  # Shoulder lift joint
            0.3 * math.sin(3 * rospy.get_time()),  # Elbow joint
            0.4 * math.sin(4 * rospy.get_time()),  # Wrist 1 joint
            0.2 * math.sin(5 * rospy.get_time()),  # Wrist 2 joint
            0.1 * math.sin(6 * rospy.get_time())  # Wrist 3 joint
        ]
        joint_pub.publish(joint_state)
        #print(f"Publishing joint states: {joint_state.position}")

        rate.sleep()

if __name__ == '__main__':
    try:
        sine_wave_joint_publisher()
    except rospy.ROSInterruptException:
        pass
