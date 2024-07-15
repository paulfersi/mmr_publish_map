### How it works

It reads the cones from the file **"cones_positions.json"** and the trajectory points from the file **waypoints.json**. The json files are located in the **save_map** package (data/ folder). 
It then publish the cones on the topic "/slam/cones_positions" and the waypoints on the topic "/planning/waypoints_all", both at 20hz.

It also reads the trajectory saved by the global planner. These points are saved in a csv file. 
The csv file contains 7 columns but only 5 are published via the **SpeedProfilePoints** message (at 20hz) :
- **x_m,y_m** for the coordinates of the speed profile point
- **psi_rad** for the steering angle 
- **vx_mps** for the speed
- **ax_mps2** for the acceleration

### How to use it

You must have previously saved the cones and the waypoints in the file via the [save_map](https://github.com/paulfersi/mmr_save_map) node.
You must have previously executed the **global planner** in order to have the correct values in the csv file.

```bash
ros2 launch publish_map publish_map_launch.py
```

@mmr_driverless