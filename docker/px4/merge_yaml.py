import argparse

import yaml


def merge_yaml_files(src_file, dest_file):
    """
    Merge the content of the source YAML file into the destination YAML file in-place.

    :param src_file: Path to the source YAML file
    :param dest_file: Path to the destination YAML file
    """
    # Read source YAML file
    with open(src_file, "r") as src:
        src_yaml = yaml.safe_load(src)

    # Read destination YAML file
    with open(dest_file, "r") as dest:
        dest_yaml = yaml.safe_load(dest)

    # Merge the contents of the source file into the destination file
    dest_yaml.update(src_yaml)

    # Write the merged content back to the destination file
    with open(dest_file, "w") as dest:
        yaml.dump(dest_yaml, dest, default_flow_style=False)


def main():
    """
    Parse command-line arguments and call the merge_yaml_files function with
    the provided file paths.
    """
    parser = argparse.ArgumentParser(description="Merge two YAML files")
    parser.add_argument("src_file", help="Path to the source YAML file")
    parser.add_argument("dest_file", help="Path to the destination YAML file")

    args = parser.parse_args()

    merge_yaml_files(args.src_file, args.dest_file)


if __name__ == "__main__":
    main()
