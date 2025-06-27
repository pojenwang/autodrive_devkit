### Autoware Aces
This is the standalone Autoware racing package `smart_gap_follow`(reactive planner) ported to `autodrive_roboracer_api` image for the `RoboRacer Sim Racing League @ ICRA 2025`

### Autodrive Sim
This Docker image requires the [AutoDRIVE Simulator](https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_sim). Use the appropriate tag for the qualification and final race maps:

  - Tag for final race: `2025-icra-compete`, tuned for Lidar topic rate of `40 - 60 Hz`  
  - Tag for qualification: `2025-icra-practice`, tuned for Lidar topic rate of `10 - 16 Hz`  

### Branches
Use the appropriate branch tuned for that map:  
  - `main` branch for final race (2025-icra-compete)  
  - `2025-icra-practice` for qualification (2025-icra-practice)  

### Building and Running our `autodrive_roboracer_api` Container
The `run.sh` script builds the Docker image and starts a container:
```sh
./docker/run.sh
```

```
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -y
```

```
colcon build
```

```
source install/setup.bash
```

### Run smart_gap_follow reactive planner
```
ros2 launch smart_gap_follow roboracer_sim.launch.xml
```
