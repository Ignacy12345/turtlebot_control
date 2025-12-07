#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class PointFollower(Node):
    def __init__(self):
        super().__init__('point_follower')

        # Publisher do sterowania TurtleBotem
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subskrypcja punktów klikniętych w oknie kamery
        self.point_sub = self.create_subscription(Point, '/point', self.point_callback, 10)

        # Aktualny punkt
        self.current_point = None

        # Domyślna wysokość okna
        self.window_height = 512

        # Timer do cyklicznego sterowania robotem
        self.timer = self.create_timer(0.1, self.control_robot)

        # Prędkość liniowa do przodu
        self.forward_speed = 0.2

    def point_callback(self, msg: Point):
        self.current_point = msg
        self.get_logger().info(f"Received point: x={msg.x:.1f}, y={msg.y:.1f}")

    def control_robot(self):
        twist = Twist()
        if self.current_point is not None:
            # Sprawdzenie, czy punkt jest powyżej lub poniżej środka okna
            if self.current_point.y < self.window_height / 2:
                twist.linear.x = self.forward_speed  # jedziemy do przodu
            else:
                twist.linear.x = 0.0  # zatrzymujemy się
        else:
            twist.linear.x = 0.0  # brak punktu → zatrzymanie

        # Publikacja komendy
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PointFollower()
    try:
        rclpy.spin(node)
    finally:
        # zatrzymanie robota przy wyłączaniu
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
