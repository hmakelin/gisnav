import argparse
import os
import xml.etree.ElementTree as ET


def merge_world_xml(source_file, merge_file):
    # Parse the source XML file
    tree = ET.parse(source_file)
    root = tree.getroot()

    # Parse the XML content to merge
    merge_tree = ET.parse(merge_file)
    merge_root = merge_tree.getroot()

    # Find the <world> element in the source XML
    world_element = root.find("world")

    # Check if <world> element exists in both source and merge content
    if world_element is not None and merge_root.find("world") is not None:
        # Merge <include> elements
        for include_element in merge_root.find("world").findall("include"):
            world_element.append(include_element)

        # Merge <spherical_coordinates> elements
        spherical_coordinates_element = merge_root.find("world").find(
            "spherical_coordinates"
        )
        if spherical_coordinates_element is not None:
            existing_spherical_coordinates = world_element.find("spherical_coordinates")
            if existing_spherical_coordinates is None:
                world_element.append(spherical_coordinates_element)
            else:
                world_element.remove(existing_spherical_coordinates)
                world_element.append(spherical_coordinates_element)

    # Write back to the source XML file
    tree.write(source_file)


def merge_camera_xml(source_file, merge_file):
    # Parse the source XML file
    tree = ET.parse(source_file)
    root = tree.getroot()

    # Parse the XML content to merge
    merge_tree = ET.parse(merge_file)
    merge_root = merge_tree.getroot()

    # Find the <link name='base_link'> element in the source XML
    base_link_element = root.find(".//link[@name='base_link']")

    # Check if <link name='base_link'> element exists in the source and merge content
    if base_link_element is not None and merge_root is not None:
        # Merge the content of merge_root into base_link_element
        # for elem in merge_root:
        #    base_link_element.append(elem)
        base_link_element.append(merge_root)

    # Write back to the source XML file
    tree.write(source_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge XML files.")
    parser.add_argument("source", type=str, help="Path to the source XML file.")
    parser.add_argument(
        "merge", type=str, help="Path to the XML file with the content to merge."
    )
    parser.add_argument(
        "--camera", action="store_true", help="Flag to indicate camera merge."
    )

    args = parser.parse_args()

    if os.path.exists(args.source) and os.path.exists(args.merge):
        if args.camera:
            merge_camera_xml(args.source, args.merge)
        else:
            merge_world_xml(args.source, args.merge)
        print("XML snippet merged successfully!")
    else:
        if not os.path.exists(args.source):
            print(f"File '{args.source}' not found.")
        if not os.path.exists(args.merge):
            print(f"File '{args.merge}' not found.")
