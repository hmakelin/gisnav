#!/usr/bin/env python3

import rclpy
import serial
from nmea_msgs.msg import Sentence
from rclpy.node import Node


class NMEAPublisherNode(Node):
    """NMEA 0183 ROS to serial bridge"""

    def __init__(self):
        super().__init__("nmea_publisher")

        # Declare and get parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value

        # Subscribe to NMEA topic
        self.subscription = self.create_subscription(
            Sentence, "/gisnav/nmea_node/sentence", self.sentence_callback, 10
        )

        self.get_logger().info(
            f"NMEA serial publisher initialized. Publishing to "
            f"{self.serial_port} at {self.baud_rate}"
        )

    def sentence_callback(self, msg) -> None:
        # Create NMEA message
        try:
            # Send NMEA message over serial using context manager
            with serial.Serial(self.serial_port, self.baud_rate) as ser:
                ser.write((msg.sentence + "\r\n").encode())

        except Exception as e:
            self.get_logger().warning(
                f"Could not write NMEA sentence {msg} to serial port "
                f"{self.serial_port} because of exception: {e}"
            )
            return None

        self.get_logger().debug(
            f"Published NMEA sentence {msg} to serial port {self.serial_port}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NMEAPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
