# Robot Simulators :robot:

## Gazebo

This project contains the files necessary to move a robot from its starting position to a goal, in this case a pioneer robot, on a map generated in a particular way based on the surnames of the author. This project is available as a git repository.

```bash
git clone -b gazebo https://github.com/Jagoca98/simuladores.git
```

## Running

```bash
git clone -b gazebo https://github.com/Jagoca98/simuladores.git
cd simuladores/gazebo-tools/model_push_g9/
mkdir -p build && cd build && cmake .. && make
cd ../..
GAZEBO_PLUGIN_PATH=model_push_g9/build gazebo --verbose map.world.xml 
```