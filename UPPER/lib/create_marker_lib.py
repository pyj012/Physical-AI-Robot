import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
class MarkerPublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "my_namespace"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD

        # 위치
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # 크기
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        # 색상 (빨간색)
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    def create_maker(self,x,y,z):
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.publisher.publish(self.marker)

        # self.get_logger().info(f"마커를 보냈습니다: x={x:.3f}, y={y:.3f}, z={z:.3f}")