Docker build contexts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the ``docker/`` folder you can find a collection of directories roughly
corresponding to :term:`Docker` build contexts, and a number of
:term:`Docker Compose` files defining services that use these build contexts. In
case of multi-stage builds multiple service images can be created from the same
build context. The ``docker/Makefile`` may define additional phony targets for
commonly needed tasks such as exposing the X server to containers that have a
:term:`GUI` component.
