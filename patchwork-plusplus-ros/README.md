# patchwork-plusplus-ros

This is a ROS2 port of Patchwork++ (@ IROS'22), which is a fast and robust ground segmentation method.

<p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

> If you are not familiar with ROS, please visit the [original repository][patchwork++link].

> If you follow the [repository][patchwork++link], you can run Patchwork++ in Python and C++ easily.

[patchwork++link]: https://github.com/url-kaist/patchwork-plusplus

## :open_file_folder: What's in this repository

* ROS2 based Patchwork source code ([patchworkpp.hpp][codelink])
* Demo launch file (`launch/demo.py`) for point cloud ground segmentation.

[codelink]: https://github.com/url-kaist/patchwork-plusplus-ros/blob/master/include/patchworkpp/patchworkpp.hpp
## :package: Prerequisite packages
You may need to install ROS2 Humble, PCL, Eigen, OpenCV, ...

## :gear: How to build Patchwork++
To build Patchwork++ in a ROS2 workspace:

```bash
$ cd /home/ubuntu/unilidar_sdk2
$ source /opt/ros/humble/setup.bash
$ colcon build --packages-select patchworkpp
$ source install/setup.bash
```

## :runner: To run the demo codes
There is a demo which executes Patchwork++ with sample rosbag file. You can download a sample file with the following command.

> For the sample rosbag data, I utilizes [semantickitti2bag](https://github.com/amslabtech/semantickitti2bag) package.

```bash
$ wget https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag
```
> If you have any trouble to download the file by the above command, please click [here][kitti_sample_link] to download the file directly.

[kitti_sample_link]: https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag

> The rosbag file is based on the [KITTI][kittilink] dataset. The bin files are merged into the rosbag file format.

> The sample file contains LiDAR sensor data only.

[kittilink]: http://www.cvlibs.net/datasets/kitti/raw_data.php

Then, you can run the ROS2 demo as follows.

```bash
# Start Patchwork++
$ ros2 launch patchworkpp demo.py

# Play a ROS2 bag that publishes /kitti/velo/pointcloud
$ ros2 bag play <converted_kitti_00_sample_bag>
```

The original `kitti_00_sample.bag` from this repository is a ROS1 bag. In a pure ROS2/Humble environment, convert it to rosbag2 first or play it through a ROS1-ROS2 bridge/plugin. The Patchwork++ demo node subscribes to `/kitti/velo/pointcloud` by default; override it with:

```bash
$ ros2 launch patchworkpp demo.py cloud_topic:=/your/pointcloud_topic
```

## :pushpin: TODO List
- [ ] Update additional demo codes processing data with .bin file format
- [ ] Generalize point type in the source code
- [ ] Add visualization result of demo codes in readme

## Citation
If you use our codes, please cite our [paper][patchwork++arXivLink].

In addition, you can also check the paper of our baseline(Patchwork) [here][patchworkarXivlink].

[patchwork++arXivLink]: https://arxiv.org/abs/2207.11919
[patchworkarXivlink]: https://arxiv.org/abs/2108.05560

```
@inproceedings{lee2022patchworkpp,
    title={{Patchwork++: Fast and robust ground segmentation solving partial under-segmentation using 3D point cloud}},
    author={Lee, Seungjae and Lim, Hyungtae and Myung, Hyun},
    booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.},
    year={2022},
    note={{Submitted}} 
}
```
```
@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
}
```

## :postbox: Contact
If you have any question, don't be hesitate let us know!

* [Seungjae Lee][sjlink] :envelope: (sj98lee at kaist.ac.kr)
* [Hyungtae Lim][htlink] :envelope: (shapelim at kaist.ac.kr)

[sjlink]: https://github.com/seungjae24
[htlink]: https://github.com/LimHyungTae
