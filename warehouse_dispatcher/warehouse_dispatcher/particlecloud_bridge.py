"""Compatibility bridge: Nav2 ParticleCloud -> geometry_msgs/PoseArray."""

from geometry_msgs.msg import PoseArray
from nav2_msgs.msg import ParticleCloud
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ParticleCloudBridge(Node):
    """Expose a PoseArray on /particlecloud for legacy/custom RViz configs."""

    def __init__(self) -> None:
        super().__init__('particlecloud_bridge')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(PoseArray, '/particlecloud', qos)
        self._sub = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self._on_cloud,
            qos,
        )
        self.get_logger().info(
            'Bridging /particle_cloud (nav2_msgs/ParticleCloud) '
            'to /particlecloud (geometry_msgs/PoseArray)'
        )

    def _on_cloud(self, msg: ParticleCloud) -> None:
        out = PoseArray()
        out.header = msg.header
        out.poses = [particle.pose for particle in msg.particles]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParticleCloudBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
