# rclpy_decorators

This package offers decorator functions that enhance the functionality of the ROS 2 Python client library, `rclpy`.
By shifting ROS boilerplate code into decorators, this package aims to make ROS application source code more readable 
and maintainable.


## Example: ROS publisher and subscriber

The code block for the `image` property below includes all the ROS boilerplate code needed to setup the publisher
and subscriber. Moving either of these properties into another Node only involves cut-and-pasting a single* code block. 
This eliminates the need for extensive back-and-forth scrolling to look for related code that is typical during 
refactoring, especially in a large Python module.

> *Currently subscriptions still require an additional setup line in e.g. the `__init__` method of a Node

```python
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from rclpy_decorators import ROS

def ExampleNode(Node):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # Call once to setup subscription
        self.image
        
    def _image_callback(self, msg: Image):
        """Callback for :attr:`.image` message
        
        Re-publishes the image via :attr:`.pub_image`
        """
        self.pub_image

    @property
    @ROS.max_delay_ms(500)
    @ROS.subscribe(
        "/camera/image_raw",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_callback,
    )
    def image(self) -> Optional[Image]:
        """Raw image data from camera
        
        :return: Image message, or None if not available or timestamp is too old
        """

    @property
    @ROS.publish(
        "~/pub_image",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pub_image(self) -> Optional[Image]:
        """Raw image data from camera, republished via this node
        
        :return: Image message, or None if not available
        """
        return self.image
```

## Example: Runtime type narrowing

The `gisnav` and `gisnav_gpu` package ROS nodes are structured in layers of computed properties that build on top of
each other, using earlier computed properties as inputs to later properties. The chain of computed
properties often builds on properties that map to ROS subscriptions and are therefore declared as `Optional` types, to
account for the possibility of the message not e.g. being received yet and having to return a `None` value.

The `narrow_types` decorator is used therefore used to de-clutter the method bodies from extensive `if ... else ...` 
statements that would otherwise be needed to check whether some computed property that derives from received ROS messages 
is available, or is `None` because not all input ROS messages have been received yet.

For example, here the decorator is used to avoid having to check whether each of the `image`, `orthoimage`, and `transform`
inputs are not None (Python type guards will not work for multiple inputs), and to log the result of the type check 
using the ROS logger, leaving the inner function body free of clutter:

```python
    ...

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_PNP_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pnp_image(self) -> Optional[Image]:
        """Published :term:`stacked <stack>` image consisting of query image,
        reference image, and reference elevation raster (:term:`DEM`).

        .. note::
            Semantically not a single image, but a stack of two 8-bit grayscale
            images and one 16-bit "image-like" elevation reference, stored in a
            compact way in an existing message type so to avoid having to also
            publish custom :term:`ROS` message definitions.
        """

        @narrow_types(self)
        def _pnp_image(
            image: Image,
            orthoimage: Image,
            transform: TransformStamped,
        ) -> Optional[Image]:
            ...

    ...
```

# License

This software is released under the MIT license. See the `LICENSE.md` file for more information.
