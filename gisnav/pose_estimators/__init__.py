"""GISNav pose estimator adapters

All pose estimators are defined in dedicated modules to keep individual file size down. They are imported here to
package namespace for convenience. For example:

.. code-block::

    #from gisnav.pose_estimators.loftr_pose_estimator import LoFTRPoseEstimator
    from gisnav.pose_estimators import LoFTRPoseEstimator
"""
from .pose_estimator import PoseEstimator
from .keypoint_pose_estimator import KeypointPoseEstimator
try:
    from .superglue_pose_estimator import SuperGluePoseEstimator
except ModuleNotFoundError as e:
    # Submodule not loaded
    pass
try:
    from .loftr_pose_estimator import LoFTRPoseEstimator
except ModuleNotFoundError as e:
    # Submodule not loaded
    pass
