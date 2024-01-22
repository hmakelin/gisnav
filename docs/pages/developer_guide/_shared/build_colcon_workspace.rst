.. tab-set::

    .. tab-item:: Entire workspace
        :selected:

        .. code-block:: bash
            :caption: Build entire workspace

            cd ~/colcon_ws
            colcon build
            source install/setup.bash

    .. tab-item:: GISNav only

        .. code-block:: bash
            :caption: Build GISNav package

            cd ~/colcon_ws
            colcon build --packages-select gisnav
            source install/setup.bash
