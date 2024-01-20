Expose your ``xhost`` to your Docker containers (see e.g.
`ROS GUI Tutorial <http://wiki.ros.org/docker/Tutorials/GUI>`_) with the
following command:

.. tab-set::

    .. tab-item:: Recommended
        :selected:

        .. code-block:: bash
            :caption: Expose xhost to Docker containers with names containing ``gisnav`` only

            for containerId in $(docker ps -f name=gisnav -aq); do
                xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' $containerId)
            done

    .. tab-item:: Easy but less safe

        .. code-block:: bash
            :caption: Expose xhost to any client

            xhost +
