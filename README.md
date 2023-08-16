You can run the main launch file like below
```ros2 launch mockup_transform_publishers transform_publisher.launch.py```

There are launch arguments which all default to true which launch files for individual pieces of mockup interaction objects. Those arguments are `use_bench`, `use_hatch_handle`, and `use_wheel`.

If you want to launch just the publishers for a single piece of the mockup, you can run the standlone files like below, and these are available for the three current supported mockup pieces.
```ros2 launch mockup_transform_publishers bench_transform_publisher.launch.py```

If you want to add any frames, add your new frames to the yaml files in the config folder for your respective mockup piece. You need these parameters. Note that the `-` makes this a vector (or list), so you need that on the first line of your new waypoint.

```
- name: bench_approach # name of the new frame
  frame_id: bench_lid # parent frame
  tx: 1.0 # translation x
  ty: 0.0 # translation y
  tz: 0.0 # translation z
  rx: 0.0 # quaternion x 
  ry: 0.0 # quaternion y
  rz: 0.0 # quaternion z 
  rw: 1.0 # quaternion w
```