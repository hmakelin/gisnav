"""Abstract base class for all nodes implementing shared logic"""
from abc import ABC, abstractmethod
from typing import Union, List, Tuple
import time

from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rcl_interfaces.msg import ParameterDescriptor


class BaseNode(ABC, Node):
    """Abstract base class for all nodes implementing shared logic

    This class is intended to be extended by nodes in the same package and should not be imported directly in any other
    package.
    """
    @property
    @abstractmethod
    def ROS_PARAM_DEFAULTS(self) -> List[Tuple[str, Union[int, float, str, bool, List[str]], bool]]:
        """List containing ROS parameter name, default value and read_only flag tuples

        The ROS parameter defaults are declared as an abstract property to encourage all implementing classes to define
        the ROS parameters they use in one place. When defined as a class constant it also gets picked up by Sphinx
        autodoc so that the list of ROS parameters for each node can be conveniently checked in the API docs.

        .. note::
            The property name is all upper-case to suggest it should be declared as class constant variable (for
            documentation reasons stated above) instead of as a property.
        """
        pass

    def __init__(self, name: str):
        """Initializes the node

        :param name: Node name
        """
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.__declare_ros_params()

    @property
    def sec(self) -> int:
        """Returns system time in seconds"""
        return time.time()

    @property
    def usec(self) -> int:
        """Returns system time in microseconds"""
        return int(time.time_ns() / 1e3)

    def __declare_ros_params(self) -> None:
        """Declares ROS parameters

        Does not override ROS parameters that are already declared (e.g. from YAML file on launch).
        """
        # Declare parameters one by one because declare_parameters will not declare remaining parameters if it
        # raises a ParameterAlreadyDeclaredException
        for param_tuple in self.ROS_PARAM_DEFAULTS:
            param, default_value, read_only = param_tuple
            try:
                self.declare_parameter(param, default_value, ParameterDescriptor(read_only=read_only))
                self.get_logger().info(f'Using default value "{default_value}" for ROS parameter "{param}".')
            except ParameterAlreadyDeclaredException as _:
                # This means parameter is already declared (e.g. from a YAML file)
                value = self.get_parameter(param).value
                self.get_logger().info(f'ROS parameter "{param}" already declared with value "{value}".')
