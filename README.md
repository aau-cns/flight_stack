# CNS Flight Stack: ROS1 Catkin Workspace

[![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{cns_flight_stack22,
   author   = {Martin Scheiber and Alessandro Fornasier and Roland Jung and Christoph Boehm and Rohit Dhakate
               and Christian Stewart and Jan Steinbrener and Stephan Weiss and Christian Brommer},
   journal  = {IEEE Robotics and Automation Letters},
   title    = {CNS Flight Stack for Reproducible, Customizable, and Fully Autonomous Applications},
   year     = {2022},
}
```

## Tutorial

## Getting Started

### Prerequesites



## Setup and compile the project

```bash
# Generate a catkin workspace (optional)
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src

# Get the ros package
git clone <url> amadee20_cws
cd amadee20_cws
git submodule update --init --recursive

# Build the project and run tests
cd ../../
catkin build
```

For further detail about the scripts and maintainance of an catkin workspace follow:
https://gitlab.aau.at/aau-nav/development/examples/example_catkin_ws
