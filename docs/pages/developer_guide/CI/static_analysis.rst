Static analysis
____________________________________________________

See ``.pre-commit-config.yaml`` and ``pyproject.toml`` files for static
analysis pipeline configuration.

You can run the pre-commit hooks with the following commands:

.. code-block:: bash

    pip install -r requirements-dev.txt
    pre-commit run --all-files

Install pre-commit as part of your own git workflow:

.. code-block:: bash

    pre-commit install
