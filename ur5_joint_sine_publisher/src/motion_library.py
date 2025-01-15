import numpy as np
import PyKDL
from scipy.spatial.transform import Rotation as R
from urdf_parser_py.urdf import URDF

class MotionPlanner:
    def __init__(self, robot_urdf):
        # Initialize the KDL solver from the robot URDF
        self.robot_urdf = robot_urdf
        self.chain = self.load_robot_chain()
        self.fk_solver = PyKDL.ForwardKinematics(self.chain)
        self.ik_solver = PyKDL.InverseKinematics(self.chain)

    def load_robot_chain(self):

        try:
            # Parse the URDF file
            with open(self.robot_urdf, 'r') as urdf_file:
                robot_urdf = URDF.from_xml_string(urdf_file.read())

            # Extract the base and end-effector links
            base_link = robot_urdf.get_root()
            end_effector_link = robot_urdf.links[-1].name  # Assume the last link is the end-effector

            # Build the KDL tree from the URDF model
            kdl_tree = PyKDL.Tree()
            for joint in robot_urdf.joints:
                if joint.type != "fixed":
                    parent_link = joint.parent
                    child_link = joint.child
                    joint_axis = PyKDL.Vector(*joint.axis)
                    joint_origin = PyKDL.Frame(PyKDL.Vector(*joint.origin.xyz))

                    kdl_tree.addSegment(
                        PyKDL.Segment(
                            child_link,
                            PyKDL.Joint(joint.name, PyKDL.Joint.RotAxis, joint_axis),
                            joint_origin
                        ),
                        parent_link
                    )

            # Extract the kinematic chain from base to end-effector
            chain = kdl_tree.getChain(base_link, end_effector_link)
            return chain

        except Exception as e:
            print(f"Error loading KDL chain: {e}")
            return None

    def joint_motion(self, point1, point2, joint_velocity, joint_acceleration):
        distance = np.linalg.norm(point2 - point1)
        total_time = 2 * (distance / joint_velocity)
        time_steps = np.linspace(0, total_time, int(total_time / 0.1) + 1)

        # Interpolate positions along the joint trajectory
        joint_trajectory = []
        for t in time_steps:
            alpha = t / total_time
            positions = (1 - alpha) * point1 + alpha * point2
            joint_trajectory.append(positions)

        return joint_trajectory

    def cartesian_motion(self, pose1, pose2, linear_velocity, linear_acceleration):
        translation1 = pose1[:3, 3]
        translation2 = pose2[:3, 3]
        distance = np.linalg.norm(translation2 - translation1)

        # Calculate the total time needed to execute the motion
        total_time = 2 * (distance / linear_velocity)  # Simple trapezoidal motion

        time_steps = np.linspace(0, total_time, int(total_time / 0.1) + 1)

        # Interpolate poses in Cartesian space
        cartesian_trajectory = []
        for t in time_steps:
            alpha = t / total_time
            interpolated_translation = (1 - alpha) * translation1 + alpha * translation2
            interpolated_pose = pose1.copy()
            interpolated_pose[:3, 3] = interpolated_translation
            cartesian_trajectory.append(interpolated_pose)

        # For each pose in the trajectory, compute the corresponding joint angles using IK
        joint_trajectory = []
        for pose in cartesian_trajectory:
            joint_angles = self.inverse_kinematics(pose)
            if joint_angles is not None:
                joint_trajectory.append(joint_angles)

        return joint_trajectory

    def inverse_kinematics(self, target_pose):

        position = PyKDL.Vector(target_pose[0, 3], target_pose[1, 3], target_pose[2, 3])
        rotation_matrix = target_pose[:3, :3]
        rotation = R.from_matrix(rotation_matrix)
        quat = rotation.as_quat()
        orientation = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])

        target_frame = PyKDL.Frame(orientation, position)

        # Use the IK solver to compute the joint angles
        joint_angles = PyKDL.JointArray(self.chain.getNrOfJoints())
        ik_success = self.ik_solver.CartToJnt(joint_angles, target_frame, joint_angles)

        if ik_success >= 0:
            return np.array([joint_angles[i] for i in range(self.chain.getNrOfJoints())])
        else:
            print("IK Solver failed.")
            return None

