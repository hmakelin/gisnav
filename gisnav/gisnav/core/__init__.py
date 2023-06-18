"""Package containing GISNav :term:`core` :term:`ROS` nodes

:class:`.CVNode` subscribes to :class:`.GISNode` published topics, so the
relative :term:`topic` names are defined here in the package ``__init__`` module
and made available to each node contained in therein.
"""

from .cv_node import CVNode
from .gis_node import GISNode

__all__ = ["CVNode", "GISNode"]
