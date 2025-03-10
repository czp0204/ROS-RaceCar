# slam\_sim\_demo

SLAM演示功能包，本演示包包含以下内容：

* **gmapping_demo**: gmapping SLAM演示
* **karto_demo**: karto SLAM演示
* **hector_demo**: hector SLAM演示
* **cartographer_demo**: Google catographer演示


maps存储地图
roslaunch slam_sim_demo gmapping_demo.launch 建图

### 注意事项
如果gazebo出现错误，比如无法查看摄像头换面，你需要升级gazebo到gazebo7及以上版本:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```
