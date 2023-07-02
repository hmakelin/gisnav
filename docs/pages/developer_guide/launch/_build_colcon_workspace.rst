.. tab-set::

    .. tab-item:: Entire workspace
        :selected:

        .. code-block:: bash
            :caption: Build entire colcon workspace

            mkdir -p ~/colcon_ws
            colcon build
            source install/setup.bash

    .. tab-item:: GISNav package only
        :selected:

        .. code-block:: bash
            :caption: Build GISNav package

            mkdir -p ~/colcon_ws
            colcon build --packages-select gisnav
            source install/setup.bash
