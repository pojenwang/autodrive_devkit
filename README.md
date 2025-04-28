### Building and Running the Container
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
colcon build --packages-select smart_gap_follow
```

```
source install/setup.bash
```