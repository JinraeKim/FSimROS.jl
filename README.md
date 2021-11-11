# FSimROS
A package of [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl) family for ROS2.

Note that `./src/fsim_interfaces` is a ROS2 package for providing interfaces of FlightSims.jl family.

## Examples
### Minimal publisher-subscriber example
See `./test/pubsub`.

### Processor in the loop simulation (PILS)
See `./test/PILS`.
For example, set position control example results in:
![Alt Text](./figures/PILS_set_position_control.mp4.gif)

## To-do
- [ ] sync issues (maybe?) for divergence of controller (which requires integration)
    - Perhaps, we need a central node for topic `time` and each simulation and controller receives the `time` to propagate own dynamical system and adaptive control system.


## Notes
- ROS2 should be installed.
- Put `./src/fsim_interfaces` in `dev_ws/src` (ROS2 convention; don't be confused with `./src`) where your ROS2 workspace is, namely, `dev_ws`.
- Tested with only few test environments, e.g., Ubuntu 20.04, ROS2 foxy, docker.
- You must properly source the appropriate workspace in every terminal.
