import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class TrajectorySender(Node):
    def __init__(self, topic_name='/trajectory'):
        super().__init__('trajectory_sender')
        self.publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.get_logger().info(f"Trajectory sender node started, publishing to '{topic_name}'")
        
        # Timer pour envoyer une trajectoire toutes les 5 secondes
        self.timer = self.create_timer(10.0, self.send_trajectory)


    def build_traj_sin_good(self, amplitude=0.5, frequency=0.2, duration=5.0, dt=0.1):
        traj = JointTrajectory()
        traj.joint_names = [
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
        ]

        num_points = int(duration / dt)
        for i in range(num_points):
            t = i * dt
            point = JointTrajectoryPoint()

            # positions de base + mouvement sinusoïdal sur l'articulation 3
            point.positions = np.array([0.0, -1.5708, 0, -1.5708, 0.0, 0.0]) # starting position
            point.positions[0] += amplitude * np.sin(2 * np.pi * frequency * t)
            point.positions[2] += amplitude * np.sin(2 * np.pi * frequency * t)

            # temps depuis le début
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            traj.points.append(point)
        return traj

    def build_traj_sin_col(self, amplitude=1, frequency=0.1, duration=5.0, dt=0.1):
        traj = JointTrajectory()
        traj.joint_names = [
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
        ]

        num_points = int(duration / dt)
        for i in range(num_points):
            t = i * dt
            point = JointTrajectoryPoint()

            # positions de base + mouvement sinusoïdal sur l'articulation 3
            point.positions = np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]) # starting position
            point.positions[0] -= amplitude * np.sin(2 * np.pi * frequency * t)

            point.positions[2] += amplitude * np.sin(2 * np.pi * frequency * t)

            # temps depuis le début
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            traj.points.append(point)
        return traj

    def send_trajectory(self):
        traj = self.build_traj_sin_col()
        self.publisher.publish(traj)
        self.get_logger().info("Trajectory published.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
