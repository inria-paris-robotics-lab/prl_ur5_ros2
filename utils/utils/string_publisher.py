from std_msgs.msg import String
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class StringPublisher(Node):
    def __init__(self):
        super().__init__("string_publisher")
        self.declare_parameter("topic_name", "")
        self.topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )
        self.declare_parameter("string_value", "")
        self.string_value = (
            self.get_parameter("string_value").get_parameter_value().string_value
        )
        self.string_publisher = self.create_publisher(
            String,
            self.topic_name,
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.publish_msg()

    def publish_msg(self):
        msg = String()
        msg.data = self.string_value
        self.string_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StringPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()