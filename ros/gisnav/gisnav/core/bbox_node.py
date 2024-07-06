"""This module contains :class:`.BBoxNode`, a ROS node that computes and
publishes a bounding box of the camera's ground-projected field of view. The
bounding box is used by :class:`.GISNode` to retrieve orthoimagery for the
vehicle's approximate global position.
"""
from typing import Final, Optional

import numpy as np
import pyproj
import rclpy
import tf2_ros
import tf_transformations
from geographic_msgs.msg import BoundingBox
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, NavSatFix
from tf2_ros.transform_broadcaster import TransformBroadcaster

from .. import _transformations as messaging
from .._decorators import ROS, narrow_types
from ..constants import (
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_MAVROS_GIMBAL_DEVICE_ATTITUDE_STATUS,
    ROS_TOPIC_MAVROS_GLOBAL_POSITION,
    ROS_TOPIC_MAVROS_LOCAL_POSITION,
    ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX,
)


class BBoxNode(Node):
    """Publishes a :class:`.BoundingBox` of the camera's ground-projected FOV

    > [!IMPORTANT] MAVLink Gimbal Protocol v2
    > If the MAVLink gimbal protocol v2 :class:`.GimbalDeviceAttitudeStatus` message is
    > available, only the ``flags`` value of 12 i.e. bit mask 1100 (horizon-locked
    > pitch and roll, floating yaw) is supported.
    """

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read-only ROS parameter descriptor"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        self.camera_info
        self.nav_sat_fix
        self.vehicle_pose
        self.gimbal_device_attitude_status

        # Needed for updating tf2 with camera to vehicle relative pose
        # and vehicle to wgs84 relative
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._set_params_cli_gimbal_control = self.create_client(
            SetParameters, "/mavros/gimbal_control/set_parameters"
        )
        self._set_params_cli_local_position = self.create_client(
            SetParameters, "/mavros/local_position/set_parameters"
        )
        self._set_params(
            (self._set_params_cli_local_position, self._set_params_cli_gimbal_control),
            Parameter(name="tf.send", type_=Parameter.Type.BOOL, value=True),
        )

        # The base_link_stabilized_frd frame is used as the parent frame for
        # the MAVROS gimbal_control plugin gimbal frames. This ensures that
        # gimbal roll and pitch stabilization is taken into account when publishing
        # the gimbal_{i} frames into tf2. The yaw is still assumed to float.
        # This corresponds to `GimbalDeviceAttitudeStatus` message `flags` value of
        # 12 i.e. bit mask 1100.
        self._set_params(
            (self._set_params_cli_gimbal_control,),
            Parameter(
                name="tf.frame_id",
                type_=Parameter.Type.STRING,
                value="base_link_stabilized_frd",
            ),
        )

    def _set_params(self, clients, param):
        """Need to publish base_link to gimbal(_0) transform to tf"""
        for cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"Waiting for parameter service {cli.srv_name}..."
                )

            # Create request to set the parameter
            req = SetParameters.Request()
            req.parameters = [param.to_parameter_msg()]

            # Call the service to set the parameter
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info(
                    f"Parameter {param.name}={param.value} set successfully "
                    f"for {cli.srv_name}"
                )
            else:
                self.get_logger().error(f"Failed to set parameter {param}")

    def _nav_sat_fix_cb(self, msg: NavSatFix) -> None:
        """Callback for the global position message from the EKF"""
        self.fov_bounding_box

    @property
    @ROS.subscribe(
        ROS_TOPIC_MAVROS_GLOBAL_POSITION,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_nav_sat_fix_cb,
    )
    def nav_sat_fix(self) -> Optional[NavSatFix]:
        """Vehicle GNSS fix from FCU, or None if unknown"""

    @property
    @ROS.subscribe(
        ROS_TOPIC_MAVROS_LOCAL_POSITION,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def vehicle_pose(self) -> Optional[PoseStamped]:
        """Vehicle pose in EKF local frame from FCU, or None unknown"""

    @property
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Subscribed camera info, or None if unknown

        Camera intrinsics from this message are needed for the FOV projection
        """

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX, QoSPresetProfiles.SENSOR_DATA.value
    )
    def fov_bounding_box(self) -> Optional[BoundingBox]:
        """Published bounding box of the camera's ground-projected FOV"""

        @narrow_types(self)
        def _fov_and_principal_point_on_ground_plane(
            transform: TransformStamped,
            camera_info: CameraInfo,
        ) -> Optional[np.ndarray]:
            """Projects camera principal point and FOV corners onto ground plane

            Assumes ground is a flat plane, does not take DEM into account

            :return: Numpy array of FOV corners and principal point projected onto
                ground plane (z=0 in EKF local frame) in following order: top-left,
                top-right, bottom-right, bottom-left, principal point. Shape is (5, 2).
                Coordinates are meters.
            """
            R = tf_transformations.quaternion_matrix(
                tuple(messaging.as_np_quaternion(transform.transform.rotation))
            )[:3, :3]

            # Camera position in LTP centered in current location (not EKF local
            # frame origin - only shares the z-coordinate!) - assume local
            # frame z is altitude AGL
            C = np.array((0, 0, transform.transform.translation.z))

            intrinsics = camera_info.k.reshape((3, 3))

            # List of image points: top-left, top-right, bottom-right, bottom-left,
            # principal point
            img_points = [
                [0, 0],
                [camera_info.width - 1, 0],
                [camera_info.width - 1, camera_info.height - 1],
                [0, camera_info.height - 1],
                [camera_info.width / 2, camera_info.height / 2],
            ]

            # Project each point to the ground
            ground_points = []
            for pt in img_points:
                u, v = pt

                # Convert to normalized image coordinates
                d_img = np.array([u, v, 1])

                try:
                    d_cam = np.linalg.inv(intrinsics) @ d_img
                except np.linalg.LinAlgError as _:  # noqa: F841
                    self.get_logger().error(
                        "Could not invert camera intrinsics matrix. Cannot"
                        "project FOV on ground."
                    )
                    return None

                # Convert direction to ENU frame
                d_enu = R @ d_cam

                # Find intersection with ground plane
                t = -C[2] / d_enu[2]
                intersection = C + t * d_enu

                ground_points.append(intersection[:2])

            return np.vstack(ground_points)

        @narrow_types(self)
        def _enu_to_latlon(
            bbox_coords: np.ndarray, navsatfix: NavSatFix
        ) -> Optional[np.ndarray]:
            """Convert EKF local frame ENU coordinates into WGS 84 coordinates

            :param bbox_coords: A bounding box in local ENU frame (units in meters)
            :param navsatfix: Vehicle global position

            :return: Same bounding box in WGS 84 coordinates
            """

            def _determine_utm_zone(longitude):
                """Determine the UTM zone for a given longitude."""
                return int((longitude + 180) / 6) + 1

            # Define the UTM zone and conversion
            proj_latlon = pyproj.Proj(proj="latlong", datum="WGS84")
            utm_zone = _determine_utm_zone(navsatfix.longitude)
            proj_utm = pyproj.Proj(proj="utm", zone=utm_zone, datum="WGS84")

            # Convert origin to UTM
            origin_x, origin_y = pyproj.transform(
                proj_latlon, proj_utm, navsatfix.longitude, navsatfix.latitude
            )

            # Add ENU offsets to the UTM origin
            utm_x = origin_x + bbox_coords[:, 0]
            utm_y = origin_y + bbox_coords[:, 1]

            # Convert back to lat/lon
            lon, lat = pyproj.transform(proj_utm, proj_latlon, utm_x, utm_y)

            latlon_coords = np.column_stack((lon, lat))
            assert latlon_coords.shape == bbox_coords.shape

            return latlon_coords

        @narrow_types(self)
        def _square_bounding_box(enu_coords: np.ndarray) -> np.ndarray:
            """Adjusts given bounding box to ensure it's square in the local frame

            Adds padding in X (easting) and Y (northing) directions to ensure
            camera FOV is fully enclosed by the bounding box, and to reduce need
            to update the reference image so often.

            :param enu_coords: A numpy array of shape (N, 2) representing ENU
                coordinates.
            :return: A numpy array of shape (N, 2) representing the adjusted
                square bounding box.
            """
            min_e, min_n = np.min(enu_coords, axis=0)
            max_e, max_n = np.max(enu_coords, axis=0)

            delta_e = max_e - min_e
            delta_n = max_n - min_n

            if delta_e > delta_n:
                # Expand in the north direction
                difference = (delta_e - delta_n) / 2
                min_n -= difference
                max_n += difference
            elif delta_n > delta_e:
                # Expand in the east direction
                difference = (delta_n - delta_e) / 2
                min_e -= difference
                max_e += difference

            # Construct the squared bounding box coordinates
            # Add padding to bounding box by expanding field of view bounding
            # box width in each direction
            padding = max_n - min_n
            square_box = np.array(
                [
                    [min_e - padding, min_n - padding],
                    [max_e + padding, min_n - padding],
                    [max_e + padding, max_n + padding],
                    [min_e - padding, max_n + padding],
                ]
            )

            assert square_box.shape == enu_coords.shape

            return square_box

        @narrow_types(self)
        def _bounding_box(
            fov_local_enu: np.ndarray,
        ) -> BoundingBox:
            """Create a :class:`.BoundingBox` message that envelops the provided
            FOV coordinates.

            :param fov_local_enu: A 4x2 numpy array where N is the number of points,
                and each row represents [longitude, latitude].

            :return: A :class:`.BoundingBox`
            """
            assert fov_local_enu.shape == (4, 2)

            # Find the min and max values for longitude and latitude
            min_lon, min_lat = np.min(fov_local_enu, axis=0)
            max_lon, max_lat = np.max(fov_local_enu, axis=0)

            # Create and populate the BoundingBox message
            bbox = BoundingBox()
            bbox.min_pt.latitude = min_lat
            bbox.min_pt.longitude = min_lon
            bbox.max_pt.latitude = max_lat
            bbox.max_pt.longitude = max_lon

            return bbox

        transform = (
            messaging.get_transform(
                self,
                "map",
                "camera",
                rclpy.time.Time(),  # self.vehicle_pose.header.stamp
            )
            if self.vehicle_pose is not None
            else None
        )

        fov_and_c_on_ground_local_enu = _fov_and_principal_point_on_ground_plane(
            transform, self.camera_info
        )
        if fov_and_c_on_ground_local_enu is not None:
            fov_on_ground_local_enu = fov_and_c_on_ground_local_enu[:4]
            bbox_local_enu_padded_square = _square_bounding_box(fov_on_ground_local_enu)
            bounding_box = _enu_to_latlon(
                bbox_local_enu_padded_square, self.nav_sat_fix
            )
            # Convert from numpy array to BoundingBox
            bounding_box = _bounding_box(bounding_box)
        else:
            bounding_box = None

        # TODO: here there used to be a fallback that would get bbox under
        #  vehicle if FOV could not be projected. But that should not be needed
        #  if everything works so it was removed from here.

        return bounding_box

    def _gimbal_device_attitude_status_cb(
        self, msg: GimbalDeviceAttitudeStatus
    ) -> None:
        """Callback for :class:`.GimbalDeviceAttitudeStatus` message

        :param msg: :class:`.GimbalDeviceAttitudeStatus` message from MAVROS
        """
        self._publish_stabilized_base_link_frame(msg.header.stamp)
        self._publish_for_gisnav_frames()
        self.fov_bounding_box

    @property
    @ROS.subscribe(
        ROS_TOPIC_MAVROS_GIMBAL_DEVICE_ATTITUDE_STATUS,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_gimbal_device_attitude_status_cb,
    )
    def gimbal_device_attitude_status(self) -> Optional[GimbalDeviceAttitudeStatus]:
        """Camera orientation from FCU, or None unknown"""

    def _publish_stabilized_base_link_frame(self, stamp) -> None:
        """Publishes ``base_link_frd_stabilized`` tf frame

        The MAVROS published gimbal_0 frame does not adjust for stabilization
        (shows up wrong in RViz when vehicle has pitch or roll i.e. when flying
        and not hovering for a copter type vehicle). The stabilized frame adjusts
        for vehicle pitch and roll.

        Assumes `GimbalDeviceAttitudeStatus` message `flags` value of
        12 i.e. bit mask 1100 (horizon-locked pitch and roll, floating yaw).
        """
        try:
            # Get the transform from map to base_link_frd
            trans = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            # Extract the yaw from the transform
            _, _, yaw = tf_transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )

            # Create a new transform with only the yaw for map to
            # base_link_frd_stabilized
            now = rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
            new_trans = TransformStamped()
            new_trans.header.stamp = now.to_msg()
            new_trans.header.frame_id = "map"
            new_trans.child_frame_id = "base_link_stabilized"
            new_trans.transform.translation.x = trans.transform.translation.x
            new_trans.transform.translation.y = trans.transform.translation.y
            new_trans.transform.translation.z = trans.transform.translation.z

            # Create a quaternion from the yaw
            q = tf_transformations.quaternion_from_euler(0, 0, yaw)
            new_trans.transform.rotation.x = q[0]
            new_trans.transform.rotation.y = q[1]
            new_trans.transform.rotation.z = q[2]
            new_trans.transform.rotation.w = q[3]

            # Publish the new transform
            self._tf_broadcaster.sendTransform(new_trans)

        except Exception as e:
            self.get_logger().warn(f"Could not transform map to base_link: {e}")

    def _publish_for_gisnav_frames(self) -> None:
        """Publishes gisnav_base_link to gisnav_camera_link transform"""

        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link", "camera", rclpy.time.Time()
            )
            transform.header.frame_id = "gisnav_base_link"
            transform.child_frame_id = "gisnav_camera_link"
            self._tf_broadcaster.sendTransform(transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warning(
                f"Could not publish gisnav_base_link to gisnav_camera_link due "
                f"to exception: {e}"
            )
