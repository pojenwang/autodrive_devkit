### Smart_Gap_Follow
This repository contains the source code used by Team `Autoware Aces` in the [RoboRacer Sim Racing League @ ICRA 2025](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-icra-2025/). It features the standalone Autoware Racing package `smart_gap_follow`, ported to [AutoDRIVE](https://autodrive-ecosystem.github.io/) `autodrive_roboracer_api` image for the time-trial simulation race.

### Autodrive Sim
This Docker image requires the [AutoDRIVE Simulator](https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_sim). Use the appropriate tag for the competition maps:

  - Tag for final race: `2025-icra-compete`, tuned for Lidar topic rate of `40 - 60 Hz`, running in `distributed mode`.   
  - Tag for qualification: `2025-icra-practice`, tuned for Lidar topic rate of `10 - 16 Hz`, running in `graphic mode`.  

### Branches
Use the following branches:  
  - `main` branch for final race (2025-icra-compete)  
  - `2025-icra-practice` branch for qualification round (2025-icra-practice)  

### Build and run our `autodrive_roboracer_api` Container
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
### Run autodrive_roboracer_api node
```
ros2 launch autodrive_roboracer bringup_graphics.launch.py
```
### Run smart_gap_follow reactive planner node
```
ros2 launch smart_gap_follow roboracer_sim.launch.xml
```
