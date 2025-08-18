import os
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            try:
                return yaml.safe_load(file)
            except yaml.YAMLError as exc:
                print(exc)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        print("Was not able to load the yaml file at " + absolute_file_path)
        return None


def load_tf_transforms(package_name, file_path, prefix=""):

    # Grab yaml transforms from file
    tf_yaml = load_yaml(package_name, file_path)
    if not tf_yaml:
        return None

    # Parse them all into static transforms
    transforms = []
    for tf in tf_yaml:
        tform = TransformStamped()
        tform.header.frame_id = prefix + tf["frame_id"]
        tform.child_frame_id = prefix + tf["name"]
        tform.transform.translation.x = float(tf["tx"])
        tform.transform.translation.y = float(tf["ty"])
        tform.transform.translation.z = float(tf["tz"])
        tform.transform.rotation.x = float(tf["rx"])
        tform.transform.rotation.y = float(tf["ry"])
        tform.transform.rotation.z = float(tf["rz"])
        tform.transform.rotation.w = float(tf["rw"])
        transforms.append(tform)

    return transforms
