* You must have either built the ``gisnav`` Docker image from local sources, or
  pulled it it from the container registry:

  .. tab-set::

      .. tab-item:: Build
          :selected:

          .. code-block:: bash
              :caption: Build GISNav Docker image

              cd ~/colcon_ws/src/gisnav/docker
              docker compose -p gisnav build gisnav

      .. tab-item:: Pull
          :selected:

          .. code-block:: bash
              :caption: Pull GISNav Docker image

              cd ~/colcon_ws/src/gisnav/docker
              docker compose -p gisnav pull gisnav
