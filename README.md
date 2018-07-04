
# vseplanner
The repo aims to provide a planning algorithm and relevant packages to enable a saliency--aware autonomous exploration task for aerial robots.
# How to build the planner
Create a new catkin workspace and clone all modules into that workspace.
```
mkdir vsep_ws && cd vsep_ws && mkdir src && cd src
git clone https://github.com/unr-arl/vseplanner
git submodule update --init --recursive
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```
We tested this repo with ROS Jade on Ubuntu 14.04, update for newer versions like ROS Kinetic will come soon.
# To run the algorithm in the Gazebo simulation environment
First, unzip the gazebo_models.zip in the planner folder into the ./gazebo folder:
```
unzip gazebo_models.zip && mv gallery_plane gallery_wall painting_wall ~/.gazebo/models
```

Then, launch the simulation file:
```
roslaunch ./src/launch/sim_vsep.launch
```

# Test with real datasets
We provided here [two datasets](https://drive.google.com/drive/folders/1tOGKk9jRMvzSVDGSbXiYpckpUN9rcgy8?usp=sharing) recored onboard of a MAV. (More details are explained [here](https://github.com/unr-arl/vsep-datasets)). To test the planner, please run the rosbags as well as launching a file:
```
rosbag play VSEP_Arena.bag --clock
roslaunch ./src/launch/mav_vsep_arena_offline.launch
```
and use a ros service command shown as below to trigger the planner.
```
rosservice call /vseplanner '{header: {stamp: now, frame_id: world}}'
```
If you use this software in a scientific publication, please cite the following paper:
```
@inproceedings{tung2018vsep,
  title={Visual Saliency-aware Receding Horizon Autonomous Exploration with Application to Aerial Robotics},
  author={Dang, Tung and Papachristos, Christos and Alexis, Kostas},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={to appear},
  year={2018},
  organization={IEEE}
}
```

# Contact

You can contact us for any question or remark:
* [Tung Dang](mailto:tung.dang@nevada.unr.edu)
* [Christos Papachristos](mailto:cpapachristos@unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)
