# CNS Flight Stack

[![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

Maintainers: [Christian Brommer](mailto:christian.brommer@aau.at), [Alessandro Fornasier](mailto:alessandro.fornasier@aau.at), and [Martin Scheiber](mailto:martin.scheiber@aau.at)

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@article{cns_flightstack22,
    title        = {Flight Stack for Reproducible and Customizable Autonomy Applications in Research and Industry},
    author       = {Scheiber, Martin and Fornasier, Alessandro and Jung, Roland and Böhm, Christoph and Dhakate, Rohit and Stewart, Christian and Steinbrener, Jan and Weiss, Stephan and Brommer, Christian},
    journal      = {IEEE Robotics and Automation Letters},
    volume       = {7},
    number       = {4},
    year         = {2022},
    doi          = {10.1109/LRA.2022.3196117},
    url          = {https://ieeexplore.ieee.org/document/9849131},
    pages        = {11283--11290}
}
```

## Tutorial

Coming soon

## Setup and compile the project

```bash
# Get the ros package
git clone https://github.com/aau-cns/flight_stack.git cns_flightstack
cd cns_flightstack
git submodule update --init --recursive

# Build the project
catkin init
catkin build
```
