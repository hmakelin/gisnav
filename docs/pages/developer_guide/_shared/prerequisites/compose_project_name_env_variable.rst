* You must set the ``COMPOSE_PROJECT_NAME`` environment variable to have the
  value ``gisnav``:

.. tab-set::

    .. tab-item:: Current shell session only
        :selected:

        .. code-block:: bash
            :caption: Set COMPOSE_PROJECT_NAME environment variable

            export COMPOSE_PROJECT_NAME=gisnav

    .. tab-item:: Persistent
        :selected:

        .. code-block:: bash
            :caption: Set COMPOSE_PROJECT_NAME environment variable

            echo "export COMPOSE_PROJECT_NAME=gisnav" >> ~/.bashrc
            source ~/.bashrc

.. note::
    The ``COMPOSE_PROJECT_NAME`` environment variable is set to prefix the
    service containers with the name ``gisnav``. This is needed to make the
    script in the :ref:`Expose xhost` step below work.
