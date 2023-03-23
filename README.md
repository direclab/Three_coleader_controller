# Three_coleader_controller

The `Three_coleader_controller` package enables an onboard  vision-based control of 3 three UAVS without relying on fixed infrastructures or GPS.

This repository contains the source code of the simulation of our article:

K. M. Kabore and S. Güler, "Distributed Formation Control of Drones With Onboard Perception," in IEEE/ASME Transactions on Mechatronics, vol. 27, no. 5, pp. 3121-3131, Oct. 2022, doi: 10.1109/TMECH.2021.3110660.


## Requirements

This package requires [Ubuntu 18.04 (Bionic)](https://releases.ubuntu.com/18.04/) and [ROS Melodic](https://wiki.ros.org/melodic).
The simulation environment is based on [Gazebo](http://gazebosim.org).
The majority of the code is written in Python 2.

## Usage
To launch three drones in Gazebo:

```bash
roslaunch Three_coleader_controller
 main.launch
```

## Citation

If you use this work in an academic context, please cite the following article:
```bibtex
@ARTICLE{9543115,
  author={Kabore, Kader Monhamady and Güler, Samet},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Distributed Formation Control of Drones With Onboard Perception}, 
  year={2022},
  volume={27},
  number={5},
  pages={3121-3131},
  doi={10.1109/TMECH.2021.3110660}}
  ```
  
  ## Acknowledgments
The entire responsibility of the article belongs to the owner of this article. The financial support received from TÜB ̇ITAK does not mean that the content of the publication is approved in a scientific sense by TÜB ̇ITAK.
