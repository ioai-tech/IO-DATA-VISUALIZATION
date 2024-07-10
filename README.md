# io_data_visualization

## install dependencies

Please refer to the dependencies in Dockerfile.

## build

```bash
catkin build
```

## run

```bash
cd $your_workspace
source ./devel/setup.bash
roslaunch io_data_visualizer io_data_visualizer.launch  # press ctrl c in terminal if you want exit it
```

In another terminal,run:

```bash
python play_data/play_data.py {trajectory folder path}
```
