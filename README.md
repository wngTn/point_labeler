# Point Cloud Labeling Tool

> This is based on the README from the original [point_labeler](https://github.com/jbehley/point_labeler) as well as the corresponding [wiki pages](https://github.com/jbehley/point_labeler/wiki)

 Tool for labeling of a single point clouds or a stream of point clouds.

<img src="https://user-images.githubusercontent.com/11506664/63230808-340d5680-c212-11e9-8902-bc08f0f64dc8.png" width=500>

 Given the poses of a KITTI point cloud dataset, we load tiles of overlapping point clouds. Thus, multiple point clouds are labeled at once in a certain area.

## Dependencies

* catkin
* Eigen >= 3.2
* boost >= 1.54
* QT >= 5.2
* OpenGL Core Profile >= 4.0
* [glow](https://github.com/jbehley/glow) (catkin package)

## Build

On Ubuntu 16.04 and 18.04, most of the dependencies can be installed from the package manager:
```bash
sudo apt install git libeigen3-dev libboost-all-dev qtbase5-dev libglew-dev catkin
```

Additionally, make sure you have [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) and the [fetch](https://github.com/Photogrammetry-Robotics-Bonn/catkin_tools_fetch) verb installed:
```bash
sudo apt install python-pip
sudo pip install catkin_tools catkin_tools_fetch empy
```

If you do not have a catkin workspace already, create one:
```bash
cd
mkdir catkin_ws
cd catkin_ws
```

```bash
mkdir src
catkin init
cd src
git clone https://github.com/ros/catkin.git
```
Clone the repository in your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/jbehley/point_labeler.git
```
Download the additional dependencies:
```bash
catkin deps fetch
```
Then, build the project:
```bash
catkin build point_labeler
```
Now the project root directory (e.g. `~/catkin_ws/src/point_labeler`) should contain a `bin` directory containing the labeler.





## Data folder structure

When loading a dataset (ATLAS), the data must be organized as follows:

<pre>
TRIAL
├── PHASE
    ├── pointclouds/          -- directory containing ".ply" files with point clouds.
    ├── labels/   [optional]  -- label directory, will be generated if not present.
├── PHASE
    ├── ...
    ├── ...
...
</pre>

The point clouds need to be merged into a single point cloud, i.e. not structured
into cn01, ..., cn0N folders, like the ATLAS exporter does.

## Usage



In the `bin` directory, just run `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ./labeler` to start the labeling tool. 

The labeling tool allows to label a sequence of point clouds in a tile-based fashion, i.e., the tool loads all scans overlapping with the current tile location.
Thus, you will always label the part of the scans that overlaps with the current tile.


In the `settings.cfg` files you can change the followings options:

<pre>

tile size: 100.0   # size of a tile (the smaller the less scans get loaded.)
max scans: 500    # number of scans to load for a tile. (should be maybe 1000), but this currently very memory consuming.
min range: 0.0    # minimum distance of points to consider.
max range: 50.0   # maximum distance of points in the point cloud.
add car points: true # add points at the origin of the sensor possibly caused by the car itself. Default: false.

</pre>

## Navigation

![](./img/naviation.png)

## Citation

If you're using the tool in your research, it would be nice if you cite our [paper](https://arxiv.org/abs/1904.01416):

```
@inproceedings{behley2019iccv,
    author = {J. Behley and M. Garbade and A. Milioto and J. Quenzel and S. Behnke and C. Stachniss and J. Gall},
     title = {{SemanticKITTI: A Dataset for Semantic Scene Understanding of LiDAR Sequences}},
 booktitle = {Proc. of the IEEE/CVF International Conf.~on Computer Vision (ICCV)},
      year = {2019}
}
```

We used the tool to label SemanticKITTI, which contains overall over 40.000 scans organized in 20 sequences.
