- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2) is a real-time mapping algorithm that uses a factor graph-based approach to optimize its result over time. It produces a finished map if you call its 'save_map' service. LIO-SAM has been modified in a few ways (such as the addition of the save_map
service).
- A grid division Python script can be found under `helper_scripts/`. See the comment at the top of the script.
- `map_publishers` contains two executables. One publishes a visualization of a provided Lanelet map. The other publishes the PCD map cells near the car.