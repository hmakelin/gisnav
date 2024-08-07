#!/usr/bin/env python3

import rclpy
import serial
from pyubx2 import GET, UBXMessage
from rclpy.node import Node
from ublox_msgs.msg import NavPVT


class UBXPublisherNode(Node):
    def __init__(self):
        super().__init__("ublox_serial_publisher")

        # Declare and get parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value

        # Subscribe to NavPVT topic
        self.subscription = self.create_subscription(
            NavPVT, "/gisnav/ublox_node/navpvt", self.navpvt_callback, 10
        )

        self.get_logger().info(
            f"UBlox Serial Publisher initialized. Publishing to {self.serial_port}"
        )

    def navpvt_callback(self, msg):
        # Create UBX message

        try:
            # Need to undo scaling in ublox_msgs/NavPVT, see note on scaling factors
            # here https://github.com/semuconsulting/pyubx2/blob/master/RELEASE_NOTES.md#release-120  # noqa: E501
            # See scaling factors for specific fields here:
            # and here https://github.com/semuconsulting/pyubx2/blob/master/src/pyubx2/ubxtypes_get.py  # noqa: E501
            self.get_logger().debug(f"Publishing msg: {msg}")
            ubx_msg = UBXMessage(
                "NAV",
                "NAV-PVT",
                GET,
                iTOW=msg.i_tow,
                year=msg.year,
                month=msg.month,
                day=msg.day,
                hour=msg.hour,
                min=msg.min,
                sec=msg.sec,
                valid=msg.valid,
                tAcc=msg.t_acc,
                nano=msg.nano,
                fixType=msg.fix_type,
                flags=msg.flags,
                flags2=msg.flags2,
                numSV=msg.num_sv,
                lon=msg.lon / 1e7,  # undo scaling
                lat=msg.lat / 1e7,  # undo scaling
                height=msg.height,
                hMSL=msg.h_msl,
                hAcc=msg.h_acc,
                vAcc=msg.v_acc,
                velN=msg.vel_n,
                velE=msg.vel_e,
                velD=msg.vel_d,
                gSpeed=msg.g_speed,
                headMot=msg.heading / 1e5,  # undo scaling
                sAcc=msg.s_acc,
                headAcc=msg.head_acc / 1e5,  # undo scaling
                pDOP=msg.p_dop,
                reserved1=msg.reserved1,
            )
        except Exception as e:
            self.get_logger().warning(
                f"Could not generate UBX NavPVT message because of exception: {e}"
            )
            return None

        # Send UBX message over serial using context manager
        with serial.Serial(self.serial_port, self.baud_rate) as ser:
            ser.write(ubx_msg.serialize())

        self.get_logger().debug("Published NavPVT message to serial port")


def main(args=None):
    rclpy.init(args=args)

    ublox_publisher = UBXPublisherNode()

    try:
        rclpy.spin(ublox_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ublox_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
