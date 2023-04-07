"""
This script merges an XML file containing a Gazebo plugin definition with a
typhoon_h480.sdf.jinja file, which is part of the PX4 SITL Gazebo plugin. The
XML file should include a plugin definition that needs to be placed under the
camera sensor element in the typhoon_h480.sdf.jinja file.

Usage:
    python merge_xml.py path/to/xml_file.xml path/to/typhoon_h480.sdf.jinja

Arguments:
    xml_file: Path to the XML file containing the plugin definition.
    sdf_jinja_file: Path to the typhoon_h480.sdf.jinja file.

The script will merge the plugin definition from the provided XML file into the
typhoon_h480.sdf.jinja file "in-place". If the camera sensor element is not
found in the typhoon_h480.sdf.jinja file, the script will print an error
message and exit.
"""
import argparse
import os

from lxml import etree


def merge_xml(xml_file, sdf_jinja_file):
    """
    Merge an XML file containing a Gazebo plugin definition into a
    typhoon_h480.sdf.jinja file.

    This function reads the plugin definition from the provided XML file and
    inserts it under the camera sensor element in the typhoon_h480.sdf.jinja file.
    The modified content is then written back to the typhoon_h480.sdf.jinja file.

    Args:
        xml_file (str): Path to the XML file containing the plugin definition.
        sdf_jinja_file (str): Path to the typhoon_h480.sdf.jinja file.

    If the camera sensor element is not found in the typhoon_h480.sdf.jinja file,
    an error message is printed and the function returns without modifying the file.
    """
    # Parse the provided XML file
    with open(xml_file, "r") as file:
        xml_content = file.read()
    snippet_tree = etree.fromstring(xml_content)

    # Parse the typhoon_h480.sdf.jinja file
    with open(sdf_jinja_file, "r") as file:
        sdf_jinja_content = file.read()
    sdf_jinja_tree = etree.fromstring(sdf_jinja_content)

    # Find the camera sensor in the typhoon_h480.sdf.jinja file
    camera_sensor = sdf_jinja_tree.xpath("//sensor[@name='camera']")

    if camera_sensor:
        # Insert the plugin from the XML file under the camera sensor element
        camera_sensor[0].append(snippet_tree.find(".//plugin"))

        # Write the modified content back to the typhoon_h480.sdf.jinja file
        with open(sdf_jinja_file, "wb") as file:
            file.write(etree.tostring(sdf_jinja_tree, pretty_print=True))
    else:
        print("Camera sensor not found in the typhoon_h480.sdf.jinja file.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge XML files.")
    parser.add_argument(
        "xml_file", type=str, help="Path to the XML file with the plugin data."
    )
    parser.add_argument(
        "sdf_jinja_file", type=str, help="Path to the typhoon_h480.sdf.jinja file."
    )

    args = parser.parse_args()

    if os.path.exists(args.xml_file) and os.path.exists(args.sdf_jinja_file):
        merge_xml(args.xml_file, args.sdf_jinja_file)
        print("XML snippet merged successfully!")
    else:
        if not os.path.exists(args.xml_file):
            print(f"File '{args.xml_file}' not found.")
        if not os.path.exists(args.sdf_jinja_file):
            print(f"File '{args.sdf_jinja_file}' not found.")
