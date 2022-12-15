Modify ROS parameters
____________________________________________________
You can provide overrides for the ROS parameters at launch by modifying the node-specific YAML files in the
``launch/params/`` folder. For example, to provide parameter overrides to :class:`.MapNode` at launch, you would modify
the ``launch/params/map_node.yaml`` file:

.. literalinclude:: ../../../../launch/params/map_node.yaml
    :caption: launch/params/map_node.yaml
    :language: yaml

You can take a look at the :py:attr:`.MapNode.ROS_PARAM_DEFAULTS` property for the hard-coded default values as well as
get an exhaustive list of all ROS parameters declared by the node (not everything is necessarily included in the YAML
file). Each node will have this property and use it if no overrides are provided in the YAML files.

.. note::
    You should at least configure your :ref:`GIS server` WMS endpoint in the ``launch/params/map_node.yaml`` file unless
    you are using the `gisnav-docker`_ ``sitl`` service, in which case the defaults should work.

    .. _gisnav-docker: https://github.com/hmakelin/gisnav-docker
