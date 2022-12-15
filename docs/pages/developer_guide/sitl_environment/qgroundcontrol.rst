QGroundControl
___________________________________________________
QGroundControl is the `recommended ground control station for PX4`_ and also `compatible with Ardupilot`_. It is needed
for controlling the drone in the SITL (software-in-the-loop) simulation.

.. _recommended ground control station for PX4: https://docs.px4.io/main/en/getting_started/px4_basic_concepts.html#qgroundcontrol
.. _compatible with Ardupilot: https://ardupilot.org/copter/docs/common-choosing-a-ground-station.html?highlight=qgroundcontrol#qgroundcontrol

**Install QGroundControl** by following the `official instructions`_.

.. _official instructions: https://docs.qgroundcontrol.com/master/en/getting_started/quick_start.html

You can then run QGroundControl from the directory where you installed it, for example:

.. code-block:: bash
    :caption: Run QGroundControl

    ~/Applications/QGroundControl.AppImage

.. note::
    You may need to change the QGroundControl app image file permissions and/or extract it before you can run it.
    Assuming you downloaded the app image to the ``~/Applications`` folder:

    .. code-block:: bash
        :caption: Change file permissions

        cd ~/Applications
        chmod +x QGroundControl.AppImage
        ./QGroundControl.AppImage

    .. code-block:: bash
        :caption: Extract and run

        cd ~/Applications
        ./QGroundControl.AppImage --appimage-extract-and-run
