# KITTI_L2C_Projection_ROS
KITTI dataset LiDAR to Camera Projection ROS Package  

## 🛠️Introduction
This ROS package is projection LiDAR to Camera for KITTI dataset.  
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/175941928-7abda682-5696-4b2b-9823-6995d51701a4.png" width="800px"></p>  
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/175943656-67518c53-7511-4440-be85-ef7563a93a6a.gif" width="800px"></p>  



## 🛠️Settings
|Settings|Version|
|:---:|:---:|
|OS|Ubuntu 20.04|
|Language|Python|
|ROS|Noetic|
|IDE|VS code|
|OpenCV|4.4.0|
|Numpy|1.22.3|  

|Hardware||
|:--:|:--:|  
|LiDAR|Velodyne 3D Laserscanner(360 degree)|
|Camera|Color Left|  
  
## 🛠️Node Graph  
<p align="center"><img src="https://user-images.githubusercontent.com/77342519/175933601-d8ac6950-416e-4ac7-9709-b5799b97600d.png" width="800px"></p> 
  
## 🛠️Before Start  
* KITTI dataset bag file  

Go to [Link](https://github.com/tomas789/kitti2bag) and make bagfile. You can use any raw data. This package has been tested with [2011_09_26_drive_0005 (0.6 GB)](http://www.cvlibs.net/datasets/kitti/raw_data.php)  

## 🛠️Test  
It is easy to test.  
**First**, move your workspace  
```
cd (your workspace name ex.catkin_ws)
cd src
```    
**Second**, clone this repository  
```
git clone https://github.com/SanghyunPark01/KITTI_L2C_Projection_ROS.git
```  
**Third**, build  
```
cd ..
catkin_make
```  
**Finally**, Run Node  
Open three terminal  

* First Terminal, Run Master node  
```
roscore
```  
* Second Terminal, Run Bagfile  
```
rosbag play -l (your rosbag name)
```  
* Third Terminal, Run Projection node  
```
rosrun l2c_projection l2c_node.py
```

## 🛠️Error Reporting  
* Using github issues  
* Or Sending me an E-mail