# Vision Project
Repo for the Vision Algorithms for Mobile Robotics Course Project 2018. In this project a simple visual odometry pipeline is created.

### How to setup this project

Download the repo using
```
git clone https://github.com/lgulich/vision_project.git
```

Initialize the Matlab path by running the file
```
init_workspace.m
```


The VO pipeline can be executed by running the file 
```
main.m
```

For changing the datasets there are parameters in the `main.m` file which can be set. With the parameter `ds` you can choose the dataset. Options are:

* `0` - KITTI
* `1` - Malaga
* `2` - Parking
* `3` - Ascento

For the dataset "Ascento" you can additionally turn on/off the wheel odometry fusion with the parameter `af`. Options are:
* `true` - Fuse visual and wheel odometry
* `false` - Only use visual odometry

For further information see the [wiki](https://github.com/lgulich/vision_project/wiki)

## Videos

All Videos can be found on YouTube under the following [link](https://www.youtube.com/playlist?list=PLontLx8LzKiKnGcEXOgwrntr8GkptaOE_).

For recording the videos the code was run on a INTEL BLABLBAL with 1 million gb ram. The code was run using `MATLAB 2018b`
