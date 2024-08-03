#!/usr/bin/env python3

import rclpy
import serial
from pyubx2 import UBXMessage
from rclpy.node import Node
from ublox_msgs.msg import NavPVT


class UBXPublisherNode(Node):
    def __init__(self):
        super().__init__("ublox_serial_publisher")

        # Declare and get parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

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
        ubx_msg = UBXMessage.parse(
            "NAV",
            "NAV-PVT",
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
            lon=msg.lon,
            lat=msg.lat,
            height=msg.height,
            hMSL=msg.h_msl,
            hAcc=msg.h_acc,
            vAcc=msg.v_acc,
            velN=msg.vel_n,
            velE=msg.vel_e,
            velD=msg.vel_d,
            gSpeed=msg.g_speed,
            headMot=msg.head_mot,
            sAcc=msg.s_acc,
            headAcc=msg.head_acc,
            pDOP=msg.p_dop,
            reserved1=msg.reserved1,
        )

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
