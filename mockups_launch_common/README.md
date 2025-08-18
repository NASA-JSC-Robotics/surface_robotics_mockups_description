# Launch Tools for Mockups

This package is a set of python tools that perform common functionality.
Currently things such as loading yaml files and creating static transform publishers for either one or many static transforms.

Create a new package that has a launch file that imports the static transform publisher, create a config folder with corresponding yaml file full of transforms in the following format:

```yaml
- name: new_frame_name1 # name of one new frame
  frame_id: parent_frame_name # parent frame
  tx: 1.0 # translation x
  ty: 0.0 # translation y
  tz: 0.0 # translation z
  rx: 0.0 # quaternion x
  ry: 0.0 # quaternion y
  rz: 0.0 # quaternion z
  rw: 1.0 # quaternion w
- name: new_frame_nam2 # name of another new frame
  frame_id: parent_frame_name # parent frame
  tx: 1.0 # translation x
  ty: 0.0 # translation y
  tz: 0.0 # translation z
  rx: 0.0 # quaternion x
  ry: 0.0 # quaternion y
  rz: 0.0 # quaternion z
  rw: 1.0 # quaternion w
```

You can add frames to the yaml and separate them into distinct logical partitions of transforms and load them all at once with a single launch file.
For example, include this in a launch file to launch a static transform publisher for each entry in a yaml entry:

```python
def generate_launch_description():
    bench_nodes = load_transforms("clr_trainer_hatch_offsets", "config/trainer_hatch_transforms/bench_transforms.yaml")
    nodes_to_launch = [
        *bench_nodes,
    ]
    return LaunchDescription(nodes_to_launch)
```

However, note that this will launch on static transform publisher node per defined transform.
To load all transforms and publish them as a single node, use the [multi_transform_static_publisher](scripts/multi_transform_static_publisher.py).
As in,

```python
def generate_launch_description():
    nodes_to_launch = []
    site_config_path = "ivr_evr_transforms"
    nodes_to_launch.append(Node(
        package="mockups_launch_common",
        executable="multi_transform_static_publisher.py",
        name="bench_transforms",
        arguments=["clr_trainer_hatch_offsets",
                   os.path.join("config/trainer_hatch_transforms/bench_transforms.yaml"),
                   ]
    ))
    return LaunchDescription(nodes_to_launch)
```
