# DiSCo-SLAM
- A distributed multi-robot SLAM example for 3 robots.
- The local SLAM used in our project is [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), please download [the modified version of LIO-SAM](https://github.com/yeweihuang/LIO-SAM.git), and add the DiSCo-SKAM folder into ```LIO-SAM\src```.


    .
    ├── ...
    ├── src                    
    │   ├── ...           
    │   └── DiSCo-SLAM                # Folder for multi-robot SLAM
    └── ...
- Code from [Scan Context]((https://github.com/irapkaist/scancontext)) is u[s]()ed for feature description.
- We use code from [PCM](https://github.com/lajoiepy/robust_distributed_mapper/tree/d609f59658956e1b7fe06c786ed7d07776ecb426/cpp/src/pairwise_consistency_maximization) 
for outlier detection.


## Dependency
- Same dependencies as [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM):
  - [ROS Melodic](http://wiki.ros.org/melodic#Installation)
  - [gtsam 4.0.2](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)
- Dependency for [Scan Context](https://github.com/irapkaist/scancontext):
  - [libnabo 1.0.7](https://github.com/ethz-asl/libnabo/releases)
  

## Dataset

- [The Park Dataset](https://drive.google.com/file/d/1B9-Rd7hd-hF_O_NDrmCrwz93ajs0dadw/view?usp=sharing)
- [KITTI 08 Dataset](https://drive.google.com/file/d/1l0bQlp40Xc90ZuviMSsevNOKNEg3i1xP/view?usp=sharing)

To run the KITTI08 dataset, change line 9 & 10 in launch/run.launch from
  ```
<rosparam file="$(find lio_sam)/config/params.yaml" command="load" />
<rosparam file="$(find lio_sam)/src/DiSCo-SLAM/config/mapfusion.yaml" command="load"/>
  ```
to
  ```  
<rosparam file="$(find lio_sam)/config/params_k.yaml" command="load" />
<rosparam file="$(find lio_sam)/src/DiSCo-SLAM/config/mapfusion_k.yaml" command="load"/>
  ```
    
## How to use

```
cd ~/catkin_ws/src
git clone 
cd ..
catkin_make
```

```
roslaunch lio_sam run.launch
rosbag play your_bag_name.bag
```

