Pose Estimators
====================================================
Pose estimation interface
____________________________________________________
GISNav uses an external deep learning service to perform the image matching and to return the resulting pose. You can
use your own pose estimator instead of the default `TorchServe`_ by returning the rotation matrix and translation
vector as JSON. See the below snippet of code where :class:`.PoseEstimationNode` handles the return value for guidance:

.. literalinclude:: ../../../../gisnav/nodes/pose_estimation_node.py
    :caption: Handling of pose estimation request
    :start-after: # region pose estimation request
    :end-before: # endregion pose estimation request
    :language: python
    :dedent:

.. _TorchServe: https://pytorch.org/serve/

.. note::
    GISNav used to come with adapters for two pose estimators - `SuperGlue`_ and the SuperGlue-inspired `LoFTR`_ - with
    LoFTR as the default adapter. Since then, the LoFTR network along with its pre- and post-processing handler has
    been included in the ``docker/torch-serve`` image. The old ``PoseEstimator`` interface has been removed from the
    GISNav ROS 2 package, along with the ``torch`` dependency.

    .. _SuperGlue: https://github.com/magicleap/SuperGluePretrainedNetwork
    .. _LoFTR: https://github.com/zju3dv/LoFTR

.. warning::
    * SuperGlue has `restrictive licensing requirements`_, while LoFTR has a `permissive
      license`_.
    * LoFTR uses SuperGlue for *optimal transport* so if you decide to customize the default implementation, make sure
      you use the *dual-softmax* version instead or otherwise SuperGlue licensing terms apply.

    .. _restrictive licensing requirements: https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE
    .. _permissive license: https://github.com/zju3dv/LoFTR/blob/master/LICENSE

SITL simulation quirks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The `KSQL Airport Gazebo model`_ buildings in the SITL simulation demo are featureless grey blocks, so any
:class:`.PoseEstimator` will most likely not use them for matching. This means any building elevation data (see
:ref:`Rasterizing vector data`) will not technically be used to improve pose estimates in the SITL simulation. The
below figure illustrates how LoFTR finds keypoints at an even density throughout the simulated drone's field of view
except on the featureless buildings.

.. _KSQL Airport Gazebo model: https://docs.px4.io/main/en/simulation/gazebo_worlds.html#ksql-airport

.. figure:: ../../../_static/img/gisnav_sitl_featureless_buildings.jpg

    LoFTR does not find keypoints on featureless buildings or terrain (SITL simulation)

Replace PoseEstimationNode
____________________________________________________
If you need to further customize how GISNav estimates the map pose (e.g. by running the deep learning inside the ROS
node to reduce latency, like GISNav used to do it before), you can also replace :class:`.PoseEstimationNode`
completely.

The :ref:`Aircraft GeoPose estimate topics` section describes the topic names and messages the GISNav's
:class:`.MockGPSNode` expects from :class:`.PoseEstimationNode`. You can create your own node that publishes these
messages and :class:`.MockGPSNode` will then be able to use them.

You would have to create your own launch file that strips :class:`.PoseEstimationNode` from the launch description to
prevent the two nodes from publishing their own estimates to the same topic. See :ref:`Launch from ROS launch file` for
more information on launch files.
