# Motion Control of Self-Driving Car for Trajectory Tracking

## Project Overview

<p align="justify">
This project was focused on control of an autonomous vehicle for trajectory tracking using CARLA Simulator. Various control algorithms were implemented in Python for accomplishing the task of lateral and longitudinal control of the vehicle.
</p>

Following is the list of implemented controllers:
- Lateral Controllers:
  - Bang-Bang Controller
  - PID Controller
  - Pure-Pursuit Controller
  - Stanley Controller
  - Proximally Optimal Predictive (POP) Controller
- Longitudinal Controllers:
  - PID Controller
  - Adaptive Longitudinal Controller (ALC)

|                  | **ALC**                                                                                                              | **PID**                                                                                                              |
|------------------|----------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|
| **Bang Bang**    | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20Bang-Bang.gif)    | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20Bang-Bang.gif)    |
| **PID**          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20PID.gif)          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20PID.gif)          |
| **Pure Pursuit** | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20Pure-Pursuit.gif) | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20Pure-Pursuit.gif) |
| **Stanley**      | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20Stanley.gif)      | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20Stanley.gif)      |
| **POP**          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/ALC%20-%20POP.gif)          | ![](https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Media/PID%20-%20POP.gif)          |

## File Description

- `Waypoints.txt` hosts the reference trajectory of the entire mission.
- `Controller.py` is used to implement the lateral and longitudinal controllers for tracking the trajectory.
- `Drive.py` hosts the simulation parameters, connects with the simulator and runs the entire motion control pipeline for autonomous trajectory tracking.
- `Live_Plotter.py` generates and updates plots of the vehicle states and trajectory in real-time.
- `Controller Performance Analysis.ipynb` is used to analyze the controller performance in terms of tracking metrics and latency.
- `Results` directory hosts the results of a complete trajectory tracking mission in form of plots and log files.

## Citation

We encourage you to cite the [following paper](https://arxiv.org/abs/2103.13240) when using the POP control algorithm for your research:

```bibtex
@eprint{Samak2021POP-Controller,
      title={Proximally Optimal Predictive Control Algorithm for Path Tracking of Self-Driving Cars}, 
      author={Chinmay Vilas Samak and Tanmay Vilas Samak and Sivanathan Kandhasamy},
      year={2021},
      eprint={2103.13240},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
