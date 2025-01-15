import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from motion_library import MotionPlanner
import numpy as np


class RobotMotionAPI:
    def __init__(self, robot_urdf, joint_state_topic="/joint_states", joint_command_topic="/joint_trajectory_command"):
        rospy.init_node('robot_motion_api')
        self.planner = MotionPlanner(robot_urdf)
        # subscriber  for joint states
        self.joint_state_subscriber = rospy.Subscriber(joint_state_topic, JointState, self.joint_state_callback)
        # Publisher
        self.joint_command_publisher = rospy.Publisher(joint_command_topic, JointTrajectory, queue_size=10)
        self.current_joint_state = None

    def joint_state_callback(self, msg):
        self.current_joint_state = np.array(msg.position)

    def get_current_joint_state(self):
        while self.current_joint_state is None:
            rospy.sleep(0.1)  # Wait for the initial joint state to be available
        return self.current_joint_state
    def move_robot_joint(self, point1, point2, joint_velocity, joint_acceleration):
        joint_trajectory = self.planner.joint_motion(point1, point2, joint_velocity, joint_acceleration)

        # Publish the trajectory to Gazebo
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names =['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        for idx, joint_points in enumerate(joint_trajectory):
            point_msg = JointTrajectoryPoint()
            point_msg.positions = joint_points.tolist()
            point_msg.time_from_start = rospy.Duration(idx * 0.1)  # 0.1s step duration
            trajectory_msg.points.append(point_msg)

        print(trajectory_msg)
        self.joint_command_publisher.publish(trajectory_msg)

    def move_robot_cartesian(self, pose1, pose2, linear_velocity, linear_acceleration):
        cartesian_trajectory = self.planner.cartesian_motion(pose1, pose2, linear_velocity, linear_acceleration)

        # Convert to joint trajectory
        joint_trajectory = []
        for pose in cartesian_trajectory:
            joint_positions = self.planner.compute_ik(pose)
            if joint_positions is not None:
                joint_trajectory.append(joint_positions)

        # Publish
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        for idx, joint_points in enumerate(joint_trajectory):
            point_msg = JointTrajectoryPoint()
            point_msg.positions = joint_points.tolist()
            point_msg.time_from_start = rospy.Duration(idx * 0.1)  # 0.1s step duration
            trajectory_msg.points.append(point_msg)

        print(trajectory_msg)

        self.joint_command_publisher.publish(trajectory_msg)


if __name__ == "__main__":
    # for Test

    robot_urdf = "/home/albin/ur5_ws/src/universal_robot/ur_description/urdf/urdf/xacro"  # Provide the URDF path
    api = RobotMotionAPI(robot_urdf)


    point1 = np.array([0, 0, 0, 0, 0, 0])  # Starting joint configuration
    point2 = np.array([1, 1, 1, 1, 1, 1])  # Ending joint configuration
    joint_velocity = 0.1
    joint_acceleration = 0.1

    api.move_robot_joint(point1, point2, joint_velocity, joint_acceleration) #joint move


    pose1 = np.array([[1, 0, 0, 0.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 0.5],
                      [0, 0, 0, 1]])

    pose2 = np.array([[1, 0, 0, 1.0],
                      [0, 1, 0, 1.0],
                      [0, 0, 1, 1.0],
                      [0, 0, 0, 1]])

    linear_velocity = 0.1
    linear_acceleration = 0.1

    api.move_robot_cartesian(pose1, pose2, linear_velocity, linear_acceleration) #cartesian move
