# Surface Robotics Mockups

This repo holds description packages for all mockups used in the iMETRO facility for surface logistics testing.
Components are kept in separate packages to make adjusting experimental setups on hardware easier.

The three primary mockups can be viewed with:

## Hatch 4040

Mockup of a small hatch and bench.
Used for hatch opening and closing, as well as logistics mobility tasks.

```bash
ros2 launch hatch_4040 view.launch.py
```

![Hatch 4040](./hatch_4040/hatch_4040.png)

## Hatch 6060

Mockup of a large hatch and bench.
Used for hatch opening and closing, as well as logistics mobility tasks.

```bash
ros2 launch hatch_4060 view.launch.py
```

![Hatch 4060](./hatch_4060/hatch_4060.png)

## Task Trainer

Mockup of a short bench container and different types of hatches mounted to a frame.
Used for storage and manipulation tasks.

```bash
ros2 launch trainer view.launch.py
```

![Task Trainer](./trainer/task_trainer.png)

## CTB

Simplified model of a cargo transfer bag (CTB).

![CTB](./ctb/ctb.png)
