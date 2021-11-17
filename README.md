# FSimROS
A package of [FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl) family for ROS2.

Note that `./src/fsim_interfaces` is a ROS2 package for providing interfaces of FlightSims.jl family.

## Examples
### Minimal publisher-subscriber example
See `./test/pubsub`.

### Simulation and visualisation with data communication via ROS2
See `./test/PILS` for a circular trajectory tracking of a hexacopter with a
backstepping position controller (500 Hz communication).
Use a docker image [ihany/ros:foxy-fsimros](https://hub.docker.com/layers/ihany/ros/foxy-fsimros/images/sha256-ec32acd18c74ae521294a90aa7616a513494959f33c6c13ee180a25b3a3a55db?context=repo), or use the `./Dockerfile`.
- Source `./dev_ws` in the docker image by `. install/setup.zsh`.
    - (Recommended) Enter a tmux session by `ts your_session_name`.
- Move to: `cd ~/.julia/dev/FSimROS`.
- On a terminal, run as `julia test/PILS/viz.jl`.
- On a new terminal, run as `julia test/PILS/simulator.jl`.
- On a new terminal, run as `julia test/PILS/controller.jl`.
- Wait other nodes. Then, on another new terminal, run as `julia test/PILS/timer.jl`
(which provides "simulation time"; to avoid communication delay due to JIT compile, it may sleep for a moment).
See the result (video speed adjusted):

![Alt Text](./figures/sim_trajectory_tracking.gif)

## To-do
- [x] sync issues (maybe?) for divergence of controller (which requires integration)
    - Perhaps, we need a central node for topic `time` and each simulation and controller receives the `time` to propagate own dynamical system and adaptive control system.
- [x] Update the docker image's building process to include FSimROS.jl
- [ ] Add an example of processor-in-the-loop simulation (PILS) with Pixhawk.


## Notes
- ROS2 should be installed.
- Put `./src/fsim_interfaces` in `dev_ws/src` (ROS2 convention; don't be confused with `./src`) where your ROS2 workspace is, namely, `dev_ws`.
- Tested with only few test environments, e.g., Ubuntu 20.04, ROS2 foxy, docker.
- You must properly source the appropriate workspace in every terminal.
