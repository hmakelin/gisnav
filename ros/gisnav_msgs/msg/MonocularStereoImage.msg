# This message represents a stereo image couple from a monocular camera.
# The two images should have differing timestamps and the same frame_id as they
# should be subsequent frames from the same monocular camera.
#
# The "query" and "reference" terminology is used for newer and older images
# as this message is intended to be used for visual odometry (VO), and
# especially for relative pose (i.e. linear and angular velocity) estimation.
sensor_msgs/Image query   # timestamp is newer
sensor_msgs/Image reference  # timestamp is older
