"""This module contains :class:`.WFSTNode`, an extension ROS node that
subscribes to and stores :attr:`.UORBNode.sensor_gps` messages in a
database via the WFS-T protocol.
"""
from typing import Final, Optional

import requests
from px4_msgs.msg import SensorGps
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .._decorators import ROS
from ..constants import ROS_TOPIC_SENSOR_GPS


class WFSTNode(Node):
    """:term:`ROS 2` node that subscribes to and stores
    :attr:`.UORBNode.sensor_gps` messages using WFS-T.
    """

    ROS_D_URL = "http://127.0.0.1:80/wfst"
    """Default value for :attr:`.wfst_url`

    When :ref:`deploying Docker Compose services <Deploy with Docker Compose>` the
    Docker DNS host name of the MapServer container ``gisnav-mapserver-1`` should be
    used in the URL. This should already be configured in the `default launch parameter
    file <https://github.com/hmakelin/gisnav/blob/master/gisnav/launch/params/gis_node.
    yaml>`_ which overrides this default value.

    Alternatively, if the service is on a different network, use the Docker host URL,
    or the URL of the reverse proxy.
    """

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once
        self.sensor_gps

        # Delete all features on startup
        self._delete_all_features()

    @property
    @ROS.parameter(ROS_D_URL, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wfst_url(self) -> Optional[str]:
        """ROS parameter value for WFS-T endpoint URL"""

    def _construct_wfst_insert(self, lon: float, lat: float) -> str:
        """Constructs a WFS-T Insert XML request

        :param lon: Longitude of the GPS point
        :param lat: Latitude of the GPS point
        :param alt: Altitude of the GPS point
        :return: XML string for WFS-T Insert request
        """
        wfst_template = f"""
            <wfs:Transaction service="WFS" version="1.1.0"
                xmlns:wfs="http://www.opengis.net/wfs"
                xmlns:gml="http://www.opengis.net/gml"
                xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                xmlns:gisnav="http://www.mapserver.org/tinyows/"
                xsi:schemaLocation="http://www.opengis.net/wfs
                                    http://schemas.opengis.net/wfs/1.1.0/wfs.xsd">
                <wfs:Insert>
                    <gisnav:position>
                        <gisnav:geom>
                            <gml:Point srsName="EPSG:4326">
                                <gml:coordinates>{lon},{lat}</gml:coordinates>
                            </gml:Point>
                        </gisnav:geom>
                    </gisnav:position>
                </wfs:Insert>
            </wfs:Transaction>
        """
        # TODO: add timestamp and altitude to schema?
        # <gisnav:timestamp>{timestamp}</gisnav:timestamp>
        # <gisnav:altitude>{alt}</gisnav:altitude>
        return wfst_template

    def _construct_wfst_delete_all(self) -> str:
        """Constructs a WFS-T Delete XML request to delete all gisnav:feature rows

        :return: XML string for WFS-T Delete request
        """
        wfst_template = """
            <wfs:Transaction service="WFS" version="1.1.0"
                xmlns:wfs="http://www.opengis.net/wfs"
                xmlns:ogc="http://www.opengis.net/ogc"
                xmlns:gisnav="http://www.mapserver.org/tinyows/"
                xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                xsi:schemaLocation="http://www.opengis.net/wfs
                                    http://schemas.opengis.net/wfs/1.1.0/wfs.xsd">
                <wfs:Delete typeName="gisnav:position">
                    <ogc:Filter>
                        <ogc:PropertyIsLike wildCard="%" singleChar="_" escapeChar="\\">
                            <ogc:PropertyName>gisnav:geom</ogc:PropertyName>
                            <ogc:Literal>%</ogc:Literal>
                        </ogc:PropertyIsLike>
                    </ogc:Filter>
                </wfs:Delete>
            </wfs:Transaction>
        """
        return wfst_template

    def _send_wfst_request(self, xml_data: str) -> bool:
        """Sends the WFS-T request to the WFS-T service

        :param xml_data: WFS-T XML string
        :return: True if request was successful, False otherwise
        """
        headers = {"Content-Type": "text/xml"}
        assert isinstance(self.wfst_url, str)
        try:
            response = requests.post(self.wfst_url, data=xml_data, headers=headers)
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f"Error sending data to back-end {e}")
            return False
        if response.status_code == 200:
            return True
        else:
            self.get_logger().error(f"WFS-T request failed: {response.text}")
            return False

    def _delete_all_features(self) -> None:
        """Deletes all gisnav:feature rows on startup"""
        wfst_xml = self._construct_wfst_delete_all()
        success = self._send_wfst_request(wfst_xml)
        if not success:
            self.get_logger().error("Failed to delete all features on startup")

    def _update_database(self, msg: SensorGps) -> None:
        """Updates the database using WFS-T with the received ROS 2 message data

        :param msg: :class:`.SensorGps` message containing data to insert
        """
        lon = msg.lon * 1e-7
        lat = msg.lat * 1e-7
        # alt = msg.alt * 1e-3
        wfst_xml = self._construct_wfst_insert(lon, lat)
        success = self._send_wfst_request(wfst_xml)
        if not success:
            self.get_logger().error(f"Failed to insert GPS data: {msg}")

    @property
    @ROS.subscribe(
        ROS_TOPIC_SENSOR_GPS,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_update_database,
    )
    def sensor_gps(self) -> Optional[SensorGps]:
        """Subscribed :attr:`.UORBNode.sensor_gps` mock GPS message"""
