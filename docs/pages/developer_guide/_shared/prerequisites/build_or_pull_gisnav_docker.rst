* You must either build the ``gisnav`` Docker image from local sources, or pull
  it from the GitHub Container Registry (GHCR):

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
