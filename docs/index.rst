#####################################
GISNav developer documentation
#####################################
Welcome to GISNav's developer documentation!

.. card:: Get started
    :link: pages/get_started.html

    A quick demonstration of GNSS-free visual navigation with GISNav

.. card:: Developer guide
    :link: pages/developer_guide/index.html

    Instructions on how to integrate GISNav with your own project and how to extend its functionality

.. card:: API documentation
    :link: pages/api_documentation/index.html

    GISNav public API reference for developers


Generate documentation
_________________________________________
To build this Sphinx documentation yourself, first :ref:`Install locally` including
the development dependencies and then run:

.. code-block:: bash
    :caption: Build Sphinx documentation

    cd ~/colcon_ws/src/gisnav
    make docs

The HTML documentation will then appear in the ``~/colcon_ws/src/gisnav/docs/_build/`` folder.

.. seealso::

    GISNav is built on ROS 2 and supports PX4 and ArduPilot. You may also be interested in their
    documentation:

    * `ROS 2 Documentation <https://docs.ros.org/>`_
    * `PX4 Autopilot User Guide <https://docs.px4.io/master/en/>`_
    * `ArduPilot Documentation <https://ardupilot.org/ardupilot/>`_

.. toctree::
   :maxdepth: 2
   :hidden:

   pages/get_started
   pages/developer_guide/index
   pages/api_documentation/index
   pages/glossary
