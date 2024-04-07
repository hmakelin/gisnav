"""Common assertions for convenience"""
import inspect
from copy import deepcopy
from functools import wraps
from typing import (
    Any,
    Callable,
    List,
    Optional,
    Tuple,
    TypeVar,
    Union,
    cast,
    get_args,
    get_origin,
    get_type_hints,
)

import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node
from std_msgs.msg import Header
from typing_extensions import ParamSpec

from . import _transformations as tf_

#: Original return type of the wrapped method
T = TypeVar("T")

#: Param specification of the wrapped method
P = ParamSpec("P")
# TODO: using ellipsis (...) for Callable arguments instead of ParamSpec in
# enforce_types because the decorated function is not expected to have the same
# ParamSpec. However, should check that the *args are the same length and
# **kwargs have the same keys?


def _is_generic_instance(value, origin_type, type_args):
    if origin_type == list:
        return isinstance(value, list) and all(
            isinstance(element, type_args[0]) for element in value
        )
    elif origin_type == dict:
        key_type, value_type = type_args
        return isinstance(value, dict) and all(
            isinstance(k, key_type) and isinstance(v, value_type)
            for k, v in value.items()
        )
    elif origin_type == tuple:
        return (
            isinstance(value, tuple)
            and len(value) == len(type_args)
            and all(
                isinstance(element, type_arg)
                for element, type_arg in zip(value, type_args)
            )
        )
    elif origin_type == Union:
        # Recurse all generic type args
        return any(
            _is_generic_instance(value, get_origin(type_arg), get_args(type_arg))
            if get_origin(type_arg) is not None
            else isinstance(value, type_arg)
            for type_arg in type_args
        )
    else:
        return any(isinstance(value, type_arg) for type_arg in type_args)


# TODO: make this work with typed dicts?
def narrow_types(
    arg: Union[Callable[..., T], Node] = None, return_value: Optional[Any] = None
):
    """
    Function decorator to narrow provided argument types to match the decorated
    method's type hints *in a ``mypy`` compatible way*. Can also be used to
    *enforce* types at runtime (which is a more exotic use case). Can be used
    on an instance method, or a static method if the ``node_instance`` argument
    is provided (None by default).

    If any of the arguments do not match their corresponding type hints, this
    decorator logs the mismatches and then returns None without executing the
    original method. Otherwise, it proceeds to call the original method with
    the given arguments and keyword arguments.

    .. warning::
        * If the decorated method can also return None after execution you will
          not be able to tell from the return value whether the method executed
          or the type narrowing failed. In this case, you specify a different
          return value using the optional input argument.

    .. note::
        This decorator is mainly used to streamline computed properties by
        automating the check for required instance properties with e.g. None
        values. It eliminates repetitive code and logs warning messages when a
        property cannot be computed, resulting in cleaner property
        implementations with less boilerplate code.

    :param arg: Node instance that provides the logger if this decorator
        is used to wrap e.g. a static or class method, of the wrapped method
        if this wraps an instance method
    :param return_value: Optional custom return value to replace default
        None if the types narrowing fails
    :return: The return value of the original method or parameter
        ``return_value`` if any argument does not match the type hints.
    """

    instance: Optional[Node] = None if not isinstance(arg, Node) else arg
    method: Optional[Callable] = arg if not isinstance(arg, Node) else None

    def inner_decorator(method):
        @wraps(method)
        def wrapper(*args, **kwargs):
            node_instance: Node = args[0] if instance is None else instance
            assert isinstance(node_instance, Node)

            type_hints = get_type_hints(method)
            signature = inspect.signature(method)
            bound_arguments = signature.bind(*args, **kwargs)
            bound_arguments.apply_defaults()

            mismatches = []
            for name, value in bound_arguments.arguments.items():
                if name in type_hints:
                    expected_type = type_hints[name]
                    origin_type = get_origin(expected_type)
                    type_args = get_args(expected_type)

                    if origin_type is None:
                        try:
                            check = isinstance(value, expected_type)
                        except TypeError as te:
                            # TODO: handle TypedDict better
                            if isinstance(value, dict) and "TypedDict" in str(te):
                                check = True
                            else:
                                raise te
                    else:
                        check = _is_generic_instance(value, origin_type, type_args)

                    if not check:
                        mismatches.append((name, expected_type, type(value)))

            if mismatches:
                mismatch_msgs = [
                    f"{name} (expected {expected}, got {actual})"
                    for name, expected, actual in mismatches
                ]
                log_msg = (
                    f"Unexpected input argument types for {method.__name__}: "
                    f"{', '.join(mismatch_msgs)}"
                )
                node_instance.get_logger().warn(log_msg)
                return return_value

            return method(*args, **kwargs)

        return cast(Callable[..., Optional[T]], wrapper)

    if method is not None:
        # Wrapping instance method
        return inner_decorator(method)

    # Wrapping static or class method
    return inner_decorator


def validate(
    condition: Callable[[], bool],
    logger_callable: Optional[Callable[[str], Any]] = None,
    custom_msg: Optional[str] = None,
):
    """
    A decorator to check an arbitrary condition before executing the wrapped function.

    If the condition is not met, the decorator optinally logs a warning message
    using the provided logger_callable, and then returns `None`. If the
    condition is met, the wrapped function is executed as normal.

    .. warning::
        * If the decorated method can also return None after execution you will
          not be able to tell from the return value whether the method executed
          or the validation failed. # TODO: the decorator should log a warning
          or perhaps even raise an error if the decorated function includes a
          None return type.

    :param condition: A callable with no arguments that returns a boolean value.
        The wrapped function will be executed if this condition evaluates to True.
    :param logger_callable: An optional callable that takes a single string
        argument, which will be called to log a warning message when the
        condition fails.
    :param custom_msg: Optional custom message to prefix to the logging message
    :return: The inner decorator function that wraps the target function.

    Example usage:

    .. code-block:: python

        import logging

        logging.basicConfig(level=logging.WARNING)
        logger = logging.getLogger(__name__)

        def my_condition():
            return False

        @validate(my_condition, logger.warning)
        def my_function():
            return "Success"

        result = my_function()
        print("Function result:", result)
    """

    def inner_decorator(func: Callable[P, T]):
        @wraps(func)
        def wrapper(*args: P.args, **kwargs: P.kwargs) -> Optional[T]:
            if not condition():
                if logger_callable:
                    logger_callable(
                        f"{custom_msg}: Validation failed for function "
                        f"'{func.__name__}'. Returning 'None'."
                    )
                return None
            return func(*args, **kwargs)

        return wrapper

    return inner_decorator


def cache_if(predicate):
    """
    A decorator to cache the return value of a property, storing the result
    with the same name as the decorated member but with a leading underscore. The
    decorator takes a callable (predicate function) as input that determines whether
    the property method will be executed or if the cached value will be returned
    instead.

    .. warning::
        You cannot use the property or method you are wrapping inside the
        predicate directly. If you need to use the cached value of the property
        you are wrapping inside the predicate, access the cached private
        attribute directly to prevent infinite recursion (e.g. self._my_property
        instead of self.my_property).

    Example usage:

    .. code-block:: python

        class MyClass:
            @property
            @cache_if(some_predicate_function)
            def my_property(self):
                # Your implementation

    :param predicate: A callable that takes the instance as its only argument and
        returns a boolean, indicating whether the property should be
        evaluated (True) or if the cached value should be returned (False)
    :type predicate: callable
    """

    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            cache_attr = f"_{func.__name__}"
            if hasattr(self, cache_attr) and not predicate(self):
                return getattr(self, cache_attr)
            else:
                result = func(self, *args, **kwargs)
                setattr(self, cache_attr, result)
                return result

        return wrapper

    return decorator


ROS_PARAM_TYPE = Union[
    str, int, float, bool, List[str], List[int], List[float], List[bool]
]
D = TypeVar("D", bound=ROS_PARAM_TYPE)


class ROS:
    """
    Decorators to get boilerplate code out of the Nodes to make it easier to
    see what comes in, what goes out, and what gets transformed.
    """

    # TODO: callback type, use typevar
    @staticmethod
    def subscribe(topic_name: str, qos, callback=None):
        """
        A decorator to create a managed attribute (property) that subscribes to a
        ROS topic with the same type as the property. The property should be an
        optional type, e.g. Optional[Altitude], where a None value indicates the
        message has not been received yet. The decorator also supports defining
        an optional callback method.

        # TODO: enforce or check optional type

        Example usage:

        .. code-block:: python

            from mavros_msgs.msg import Altitude
            from typing import Optional

            from . import messaging

            class AutopilotNode:
                ...

                @property
                @ROS.subscribe(messaging.ROS_TOPIC_TERRAIN_ALTITUDE, 10)
                def terrain_altitude(self) -> Optional[Altitude]:
                    pass

                def on_message(self, msg: Altitude):
                    pass

        In this example, the ``terrain_altitude`` property subscribes to the
        ``messaging.ROS_TOPIC_TERRAIN_ALTITUDE`` ROS topic. The ``on_message``
        callback will be executed every time a new Altitude message is received.

        :param topic_name: The name of the ROS topic to subscribe to.
        :param qos: The Quality of Service settings for the topic subscription.
        :param callback: An optional callback method to be executed when a new
            message is received.
        :return: A property that holds the latest message from the specified ROS
            topic, or None if no messages have been received yet.
        """

        def decorator_property(func):
            @wraps(func)
            def wrapper(self):
                """
                Wrapper function for the property.

                :param self: The instance of the class the property belongs to.
                :return: The value of the property.
                """
                cached_property_name = f"_{func.__name__}"
                cached_subscription_name = f"{cached_property_name}_subscription"

                if not hasattr(self, cached_subscription_name):

                    def _on_message(message):
                        setattr(self, cached_property_name, message)
                        if callback:
                            callback(self, message)

                    optional_type = get_type_hints(func)["return"]
                    topic_type = get_args(optional_type)[
                        0
                    ]  # brittle? handle this better
                    subscription = self.create_subscription(
                        topic_type,
                        topic_name,
                        _on_message,
                        qos,
                    )
                    setattr(self, cached_subscription_name, subscription)

                # return getattr(self, cached_property_name, func(self))
                return getattr(self, cached_property_name, None)

            return wrapper

        return decorator_property

    # TODO: use default topic name, e.g. "~/message_type"?
    # TODO: add type hints, see subscribe decorator, use TypeVar("M") below?
    @staticmethod
    def publish(topic_name: str, qos):
        """
        A decorator to create a managed attribute (property) that publishes its
        value over a ROS topic whenever it's called.

        :param topic_name: The name of the ROS topic to publish to.
        :param qos: The Quality of Service settings for the topic publishing.
        :return: A property that publishes its value to the specified ROS topic
            whenever the property is called
        """

        def decorator_property(func):
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                """
                Wrapper function for the property.

                :param self: The instance of the class the property belongs to.
                :return: The value of the property.
                """
                value = func(self, *args, **kwargs)
                cached_publisher_name = f"_{func.__name__}_publisher"

                if not hasattr(wrapper, cached_publisher_name):
                    optional_type = get_type_hints(func)["return"]
                    if get_origin(optional_type) is not None:
                        topic_type = get_args(optional_type)[
                            0
                        ]  # brittle? handle this better
                    else:
                        topic_type = optional_type
                    publisher = self.create_publisher(
                        topic_type,
                        topic_name,
                        qos,
                    )
                    setattr(wrapper, cached_publisher_name, publisher)

                if value is not None:
                    getattr(wrapper, cached_publisher_name).publish(value)

                return value

            # return property(wrapper)
            return wrapper

        return decorator_property

    @staticmethod
    def transform(
        child_frame_id: Optional[str] = None,
        invert: bool = True,
    ):
        """
        A decorator to wrap a method that returns a PoseStamped or a
        TransformStamped message, converts it to a TransformStamped
        if needed, and then publishes it on the tf topic.

        :param child_frame_id: Name of child frame
        :param invert: Set to False to not invert the transform relative to the pose
        :return: A method that publishes its return value to the tf topic whenever
            called. The return value must be TransformStamped, PoseStamped, or None
        """

        def decorator(func):
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                """
                Wrapper function for the method.

                :param self: The instance of the class the method belongs to.
                :return: The original return value of the method.
                """
                obj = func(self, *args, **kwargs)

                type_hints = get_type_hints(func)
                optional_type = type_hints["return"]
                topic_type = get_args(optional_type)[0]
                if topic_type not in (
                    None,
                    TransformStamped,
                    PoseStamped,
                    PoseWithCovarianceStamped,
                ):
                    raise ValueError(
                        f"Return type must be None, TransformStamped, PoseStamped "
                        f"or PoseWithCovarianceStamped. Detected {topic_type}"
                    )

                # Check if the broadcaster is already created and cached
                cached_broadcaster_name = "_tf_broadcaster"
                if not hasattr(self, cached_broadcaster_name):
                    broadcaster = tf2_ros.TransformBroadcaster(self)
                    setattr(self, cached_broadcaster_name, broadcaster)

                if obj is None:
                    return None
                elif isinstance(obj, PoseStamped) or isinstance(
                    obj, PoseWithCovarianceStamped
                ):
                    transform = tf_.pose_to_transform(
                        deepcopy(obj), child_frame_id=child_frame_id
                    )
                else:
                    assert isinstance(obj, TransformStamped)
                    transform = obj

                if invert:
                    transform.transform.translation.x = (
                        -transform.transform.translation.x
                    )
                    transform.transform.translation.y = (
                        -transform.transform.translation.y
                    )
                    transform.transform.translation.z = (
                        -transform.transform.translation.z
                    )
                    transform.transform.rotation.x = -transform.transform.rotation.x
                    transform.transform.rotation.y = -transform.transform.rotation.y
                    transform.transform.rotation.z = -transform.transform.rotation.z
                    # Leave rotation.w unchanged
                    transform.child_frame_id = transform.header.frame_id
                    transform.header.frame_id = child_frame_id

                # Publish the transform
                getattr(self, cached_broadcaster_name).sendTransform(transform)

                return obj  # return original object (could be Pose), not the Transform

            return wrapper

        return decorator

    @staticmethod
    def max_delay_ms(max_time_diff_ms: int):
        """
        A decorator that checks the property's ROS header timestamp and compares
        it to the current clock. If the time difference is larger than what is
        provided to the decorator (in milliseconds), the decorated function logs a
        warning and returns None instead.

        :param max_time_diff_ms: Maximum allowed time difference in milliseconds.
        :return: The wrapped function or method with added timestamp checking. The
            decorated function returns None if the timestamp is too old.
        """

        class HasHeader:
            """
            Dummy class representing any ROS message class that should have the
            header attribute
            """

            header: Header

        M = TypeVar("M", bound=HasHeader)

        def _timestamp_diff_in_milliseconds(ts1, ts2):
            # Convert the timestamps to milliseconds
            ts1_ms = ts1.sec * 1000 + ts1.nanosec / 1e6
            ts2_ms = ts2.sec * 1000 + ts2.nanosec / 1e6

            # Compute the difference between the two timestamps in milliseconds
            diff_ms = ts2_ms - ts1_ms

            return diff_ms

        def decorator(func: Callable[[Node], M]) -> Callable[[Node], Optional[M]]:
            @wraps(func)
            def wrapper(self: Node) -> Optional[M]:
                """
                Wrapper function for the property.

                :param self: The instance of the :class:`rclpy.Node` the property
                    belongs to.
                :return: The value of the property if the time difference is within
                    the allowed limit or None otherwise.
                """
                message = func(self)
                if message is None:
                    return None

                if hasattr(message, "header"):
                    header_timestamp = message.header.stamp
                    current_timestamp = self.get_clock().now().to_msg()
                    time_diff = _timestamp_diff_in_milliseconds(
                        header_timestamp, current_timestamp
                    )

                    if time_diff > max_time_diff_ms:
                        self.get_logger().warn(
                            f"Time difference for message {type(message)} "
                            f"({time_diff} ms) in {func.__name__} exceeded allowed "
                            f"limit ({max_time_diff_ms} ms)."
                        )
                        return None
                else:
                    self.get_logger().warn(
                        f"Message of type {type(message)} did not have a header. "
                        f"Assuming it is not too old."
                    )

                return message

            # return property(wrapper)
            return wrapper

        return decorator

    @staticmethod
    def setup_node(params: List[Tuple[str, Any, bool]]):
        """
        A decorator that declares ROS parameters for a given class. The parent
        Node initialization is handled inside this decorator, so the child node
        should not call the parent initializer.

        .. warning::
            The parameters declared by this decorator will not be available
            until after class instantiation. Do not try to use them in the
            __init__ method.

        :param params: A list of tuples containing ROS parameter name,
            default value, and optional read-only flag.
        :type params: List[Tuple[str, Union[int, float, str, bool, List[str]], bool]]

        Example usage:

        .. code-block:: python

            class MyClass(Node):

                @ROS.setup_node([
                    ("param1", 1, True),
                    ("param2", 2),
                    ("param3", "default_value"),
                ])
                def __init__(self):
                    # Handled by decorator - do not call parent constructor here
                    # super().__init__("my_node")
                    pass

        """

        def decorator(initializer):
            @wraps(initializer)
            def wrapped_function(node_instance, *args, **kwargs):
                # TODO: assumes name is first arg, brittle, make into kwarg?

                # Call rclpy Node constructor
                Node.__init__(
                    node_instance,
                    node_name=args[0],
                    *args[1:],
                    **kwargs,
                    allow_undeclared_parameters=True,
                    automatically_declare_parameters_from_overrides=True,
                )
                for param_tuple in params:
                    param, default_value, *extras = param_tuple
                    read_only = extras[0] if extras else False
                    descriptor = ParameterDescriptor(read_only=read_only)

                    try:
                        node_instance.declare_parameter(
                            param, default_value, descriptor
                        )
                        node_instance.get_logger().info(
                            f'Using default value "{default_value}" for ROS '
                            f'parameter "{param}".'
                        )
                    except ParameterAlreadyDeclaredException:
                        value = node_instance.get_parameter(param).value
                        node_instance.get_logger().info(
                            f'ROS parameter "{param}" already declared with '
                            f'value "{value}".'
                        )

                # Call child constructor after declaring params
                initializer(node_instance, *args, **kwargs)

            return wrapped_function

        return decorator

    # List of types supported by ROS2
    SUPPORTED_ROS_TYPES = (
        str,
        int,
        float,
        bool,
        List[str],
        List[int],
        List[float],
        List[bool],
    )

    @staticmethod
    def parameter(
        default_value: ROS_PARAM_TYPE,
        descriptor: Optional[ParameterDescriptor] = None,
    ) -> Callable[
        [Callable[..., Optional[ROS_PARAM_TYPE]]],
        Callable[..., Optional[ROS_PARAM_TYPE]],
    ]:
        """
        Decorator for declaring a property as a ROS parameter in a ROS node.
        It uses the name of the property for the parameter name and the return
        type hint for the parameter type.

        :param default_value: Default value for parameter
        :param descriptor: Optional parameter descriptor
        :return: Decorator function
        :raises ValueError: If the decorator is not used in a ROS node
        """

        def decorator(
            func: Callable[..., Optional[ROS_PARAM_TYPE]]
        ) -> Callable[..., Optional[ROS_PARAM_TYPE]]:
            param_name = func.__name__
            param_type = inspect.signature(func).return_annotation

            @wraps(func)
            def wrapper(self: Node) -> Optional[ROS_PARAM_TYPE]:
                """
                Wrapper function that declares a ROS parameter and sets a callback
                for parameter changes

                :param self: Instance of the class (ROS node)
                :return: Result of the decorated function
                :raises ValueError: If the decorator is not used in a ROS node
                """
                if not isinstance(self, Node):
                    raise ValueError("ROS parameter can only be declared in a ROS node")

                try:
                    # Attempt to describe the parameter
                    # self.describe_parameter(param_name)
                    param_value = self.get_parameter(param_name).value
                    if param_value is None:
                        # Need to do this manually since allow_undeclared_parameters
                        # might be enabled
                        raise ParameterNotDeclaredException(param_name)

                    origin_type = get_origin(param_type)
                    type_args = get_args(param_type)
                    if (
                        origin_type is not None
                        and not _is_generic_instance(
                            param_value, origin_type, type_args
                        )
                        or origin_type is None
                        and not isinstance(param_value, param_type)
                    ):
                        self.get_logger().error(
                            f"Return value of {param_name} get_parameter() "
                            f"{param_value} does not match declared type return "
                            f"type {param_type}."
                        )
                        return None
                    else:
                        return param_value
                except ParameterNotDeclaredException:
                    # Parameter not declared yet, so we declare it now
                    self.get_logger().info(
                        f'Using default value "{default_value}" for ROS '
                        f'parameter "{param_name}".'
                    )
                    if descriptor is None:
                        self.declare_parameter(param_name, default_value)
                    else:
                        self.declare_parameter(param_name, default_value, descriptor)
                    return default_value

            return wrapper

        return decorator

    @staticmethod
    def retain_oldest_header(func: Callable[..., Any]) -> Callable[..., Any]:
        """Decorator to ensure that the output :term:`ROS` message's timestamp
        inherits the oldest timestamp from the input ROS messages.

        The decorated function is expected to process input in the form of ROS
        messages and produce an output, which is another ROS message.

        This decorator assumes:
        1. Any input argument with a `header` attribute also has a `stamp`
           attribute within the header.
        2. The output of the decorated function has a `header` attribute with a
           `stamp` attribute.

        :param func: The function to be decorated.
        :returns: The wrapped function.
        """

        # TODO: it would be better to just retain oldest timestamp, and leave
        #  header creation explicit (e.g. frame_id might not be same as in
        #  input messages)
        @wraps(func)
        def wrapper(*args, **kwargs) -> Optional[Any]:
            # Get all ROS message headers from the inputs
            headers = [arg.header for arg in args if hasattr(arg, "header")]

            # If there are headers, find the one with oldest timestamp
            # Use timestamp seconds, ignore nanoseconds
            if headers:  # empty list evaluates to False
                oldest_header = min(
                    headers,
                    key=lambda header: header.stamp.sec,
                    default=None,
                )
            else:
                oldest_header = None

            # Call the original function
            result = func(*args, **kwargs)

            # If result is not None and we found a timestamp, set it in the result
            if (
                result is not None
                and oldest_header is not None
                and hasattr(result, "header")
            ):
                result.header = oldest_header

            return result

        return wrapper
