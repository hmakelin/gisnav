* You must set the ``COMPOSE_PROJECT_NAME`` environment variable to have the
  value ``gisnav`` at minimum for the shell that you are running
  ``docker compose`` commands from:

  .. tab-set::

      .. tab-item:: Current shell only
          :selected:

          .. code-block:: bash
              :caption: Set ``COMPOSE_PROJECT_NAME`` environment variable

              COMPOSE_PROJECT_NAME=gisnav

      .. tab-item:: Current session

          .. code-block:: bash
              :caption: Set ``COMPOSE_PROJECT_NAME`` environment variable

              export COMPOSE_PROJECT_NAME=gisnav

      .. tab-item:: Persistent

          .. code-block:: bash
              :caption: Set ``COMPOSE_PROJECT_NAME`` environment variable

              echo "export COMPOSE_PROJECT_NAME=gisnav" >> ~/.bashrc
              source ~/.bashrc

  .. note::
      The ``COMPOSE_PROJECT_NAME`` environment variable is set to prefix the
      service containers with the name ``gisnav``. This is needed by the
      script that exposes the X server to find and expose X server to GISNav
      containers only (for better security).
