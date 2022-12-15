Generate code coverage reports
____________________________________________________
To generate and inspect code coverage you can use ``coverage.py``. See the
`official instructions <https://coverage.readthedocs.io/en/6.4.1/source.html>`_ on how to configure what source files
to measure:

.. code-block:: bash
    :caption: Run and inspect code coverage report for ROS launch tests for PX4 (Fast DDS) launch configuration

    cd ~/colcon_ws
    python3 -m coverage run --branch --include */site-packages/gisnav/* src/gisnav/test/test_px4_launch.py
    python3 -m coverage report
