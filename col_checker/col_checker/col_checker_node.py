import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory

from pinocchio_ros2.robot import Robot
from pinocchio_ros2.tools.utils import replace_path_to_absolute




class Traj_exec(Node):
    ROBOT_CONTROLLED = "left_arm"
    def __init__(self,traj_topic='/trajectory', command_topic='/left_joint_trajectory_controller/joint_trajectory'):
        super().__init__('glue_spread')
        self.get_logger().info('Col checker node has been started.')

        ## Create Pinocchio Robot
        urdf = replace_path_to_absolute("prl_ur5_description", "urdf/mantis.urdf.xacro")
        srdf = replace_path_to_absolute("prl_ur5_moveit", "config/mantis.srdf")
        self.robot = Robot(self, urdf, srdf, "joint_states")
        self.get_logger().info("Pinocchio Robot initialized")


        self.obs_traj = self.create_subscription(JointTrajectory, traj_topic, self.traj_callback, 10)
        self.traj_publisher = self.create_publisher(JointTrajectory, command_topic, 10)
        self.get_logger().info("Node initialized, waiting for trajectories...")

    def collision_check(self, q_col):
        """
        Check for collisions during the trajectory execution.
        Args:
            q_col (np.ndarray): Current configuration of the robot.
        Returns:
            Tuple[bool, List[Tuple[int, int]]]: Collision result and list of colliding pairs.
        """
        q_complete = self.robot.get_meas_qvtau()[0]
        if (self.ROBOT_CONTROLLED == "left_arm"):
            q_complete[0:len(q_col)] = q_col
        else:
            q_complete[:len(q_col)] = q_col
        col_res, col_pairs = False, []
        col_res, col_pairs = self.robot.compute_collisions_broadphase(q_complete, True)
        return col_res, col_pairs

    def check_trajectory(self,traj_to_check):
        """
        Check the entire trajectory for collisions.
        Returns:
            bool: True if the trajectory is collision-free, False otherwise.
        """
        self.get_logger().info(f"Checking trajectory with {len(traj_to_check.points)} points for collisions...")

        if not hasattr(self, 'robot') or self.robot is None:
            self.get_logger().error("Robot not initialized yet.")
            return False

        if traj_to_check is None:
            self.get_logger().warning("No trajectory to check.")
            return False
        poses = [np.array(point.positions) for point in traj_to_check.points]

        for i, pose in enumerate(poses):
            col_res, col_pairs = self.collision_check(pose)
            if col_res:
                self.get_logger().warning(f"Collision detected at pose {i}: {col_pairs}")
                return False
        self.get_logger().info("No collisions detected in the trajectory.")
        return True
    

    def traj_callback(self, msg):
        """
        Callback to receive and store the trajectory.
        Args:
            msg (Traj): Incoming trajectory message.
        """
        self.get_logger().info(f"Trajectory received with {len(msg.points)} points.")

        traj_received = msg

        if len(traj_received.points) == 0:
            self.get_logger().warning("Received empty trajectory.")
            return
        if self.check_trajectory(traj_received):
            self.get_logger().info("New trajectory accepted and processing started.")
            self.traj_publisher.publish(traj_received)
        else:
            self.get_logger().error("New trajectory rejected due to collisions.")

def main(args=None):
    rclpy.init(args=args)
    node = Traj_exec()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
