Modify ROS parameters
____________________________________________________

When deploying GISNav :ref:`with Make <Deploy using Makefile>`,
:ref:`with Docker Compose <Deploy with Docker Compose>`, :ref:`using the
ROS 2 launch system <Use ROS 2 launch system>`, or :ref:`when running individual
ROS nodes <Run individual ROS nodes>`, you can provide overrides for the ROS
parameters at launch by modifying the node-specific YAML files in the
``launch/params/`` folder. For example, to provide parameter overrides to
:class:`.GISNode` at launch, you would modify the ``launch/params/gis_node.yaml``
file:

.. literalinclude:: ../../../../gisnav/launch/params/gis_node.yaml
    :caption: :class:`.GISNode` ROS parameter configuration at launch
    :language: yaml

See the :class:`.GISNode` API reference for the
hard-coded default values as well an exhaustive list of all ROS parameters
declared by the node (not everything is necessarily included in the YAML
file).

.. note::
    You should at least configure your :ref:`GIS server` WMS endpoint ``wms_url``
    in the ``launch/params/gis_node.yaml`` file unless you are using the ``mapserver``
    Docker Compose service, in which case the defaults should work.
