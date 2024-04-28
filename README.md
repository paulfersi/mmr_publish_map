### How it works

It reads the cones from the file **"cones_positions.json"** located in the **save_map** package (data/ folder), and publish them on the topic "slam/cones_positions" at 20Hz.

### How to use it

You must have previously saved the cones in the file via the **save_map** node.

```bash
ros2 run publish_map runner
```