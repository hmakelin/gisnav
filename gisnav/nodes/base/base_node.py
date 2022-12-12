"""Abstract base class for all nodes implementing shared logic"""
from abc import ABC
from typing import Union, List, Tuple
import time

from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException


class _BaseNode(ABC, Node):
    """Abstract base class for all nodes implementing shared logic

    This class is intended to be extended by nodes in the same package and should not be imported directly in any other
    package.
    """
    def __init__(self, name: str, ros_param_defaults: dict = None):
        """Initializes the node

        :param name: Node name
        :param ros_param_defaults: ROS parameters and default values to declare on initialization
        """
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        if ros_param_defaults is not None:
            self.__declare_ros_params(ros_param_defaults)

    @property
    def sec(self) -> int:
        """Returns system time in seconds"""
        return time.time()

    @property
    def usec(self) -> int:
        """Returns system time in microseconds"""
        return int(time.time_ns() / 1e3)

    def __declare_ros_params(self, ros_params: List[Tuple[str, Union[int, float, str, bool, List[str]]]]) -> None:
        """Declares ROS parameters

        Does not override ROS parameters that are already declared (e.g. from YAML file on launch).

        :param ros_params: List of tuples containing ROS parameter name and default value
        """
        # Declare parameters one by one because declare_parameters will not declare remaining parameters if it
        # raises a ParameterAlreadyDeclaredException
        for param_tuple in ros_params:
            param, default_value = param_tuple
            try:
                self.declare_parameter(param, default_value)
                self.get_logger().info(f'Using default value "{default_value}" for ROS parameter "{param}".')
            except ParameterAlreadyDeclaredException as _:
                # This means parameter is already declared (e.g. from a YAML file)
                value = self.get_parameter(param).value
                self.get_logger().info(f'ROS parameter "{param}" already declared with value "{value}".')
