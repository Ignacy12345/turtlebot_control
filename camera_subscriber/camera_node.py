#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np

class PointPublisherNode(Node):
    def __init__(self):
        super().__init__('point_publisher_node')

        # Parametr długości kwadratu
        self.declare_parameter('square_size', 200)
        self.square_size = self.get_parameter('square_size').get_parameter_value().integer_value

        # Publisher Point na temat /point
        self.publisher = self.create_publisher(Point, '/point', 10)

        # Subskrypcja obrazu (opcjonalnie, można zostawić pustą tablicę)
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)

        self.window_name = "Camera Window"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.cv_image = np.zeros((512, 700, 3), np.uint8)
        self.point = None

    def listener_callback(self, msg):
        # Tu można konwertować obraz z kamery, jeśli jest dostępny
        # Na razie zostawiamy czarną tablicę
        pass

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)
            # Publikacja punktu
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info(f'Published point: x={x}, y={y}')

    def draw_square(self):
        img = self.cv_image.copy()
        if self.point is not None:
            cv2.rectangle(
                img,
                self.point,
                (self.point[0] + self.square_size, self.point[1] + self.square_size),
                (0, 255, 0),
                3
            )
        cv2.imshow(self.window_name, img)

def main(args=None):
    rclpy.init(args=args)
    node = PointPublisherNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.draw_square()
            if cv2.waitKey(25) & 0xFF == 27:  # ESC zamyka okno
                break
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
