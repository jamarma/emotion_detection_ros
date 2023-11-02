# Real Time Emotion Detection for Low Cost Robot in ROS

## Overview

This is a ROS package developed for **emotion detection** in camera images of **low cost robots**.
The detection system has been developed in Javier Martínez's final degree project, more information [here](https://github.com/jmvega/tfg-jmartinez).

:page_facing_up: You can find an [article published in Electronics MDPI](https://www.mdpi.com/2030510) with all the development and experiments of the project :page_facing_up:

This package has been tested under **ROS Noetic** in **Raspberry Pi 4b** under **Raspberry Pi OS (buster) 32-bit**. The camera that has been used is the **Raspberry Pi Camera Module V2.1**. 

Author: [Javier Martínez](https://github.com/jmrtzma)

Based on **The Extended Cohn-Kanade Dataset (CK+)**[1][2] this system can detect 4 emotions:

* Happy
* Surprise
* Anger
* Sadness

There are three models pre-trained available for use:

* **KNN** (K Nearest Neighbours) - Accuracy: 95% (model recommended)
* **SVM** (Support Vector Machine) - Accuracy: 95% (value of probability in output not available)
* **MLP** (Multi Layer Perceptron) - Accuracy: 95%

In [Model configuration](https://github.com/jmrtzma/emotion_detection_ros#model-configuration) section explains how to change the model.

### One face examples

![](https://github.com/jmrtzma/emotion_detection_ros/blob/main/doc/1_face.png)

### Two face examples

![](https://github.com/jmrtzma/emotion_detection_ros/blob/main/doc/2_face.png)

### Performance
We have measured the system performance on the Raspberry Pi 4b which depends on the configuration parameter `max_num_faces` ([Model configuration](https://github.com/jmrtzma/emotion_detection_ros#model-configuration) section).

* `max_num_faces: 1`

![](https://github.com/jmrtzma/emotion_detection_ros/blob/main/doc/performance1.png)

* `max_num_faces: 2`

![](https://github.com/jmrtzma/emotion_detection_ros/blob/main/doc/performance2.png)

* `max_num_faces: 3`

![](https://github.com/jmrtzma/emotion_detection_ros/blob/main/doc/performance3.png)

## Installation

### Python environment setup

We tested this system under **Python 3.7.3**. In **Raspberry Pi 4b** under **Raspberry Pi OS (buster) 32-bit**, you should change the default version of Python (Python2) to Python3.

```
sudo rm /usr/bin/python
sudo ln -s /usr/bin/python3 /usr/bin/python
```

We recommend installing the python modules in a virtual environment for version control.

```
python3 -m venv venv_name
```

Install the python modules using pip.

```
pip install -r requirements.txt
```

The module `mediapipe-rpi4` in requirements.txt file only works in Raspberry Pi 4, if you want to run this package on another platform you must install the compatible mediapipe module. Normally it is `mediapipe`.

### ROS setup

Clone this repository in your ROS workspace.
```
cd catkin_ws/src
git clone https://github.com/jmrtzma/emotion_detection_ros.git
```

Set permission to executable
```
chmod 755 emotion_detection_ros/emotion_detection_ros/scripts/emotion_detection_node.py
```

Build your ROS workspace
```
cd ..
catkin_make
```

### Raspberry Pi Camera Module V2 setup
In order to use this camera we have used the ros package [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node). Clone it in your ROS workspace (src folder).

You must also clone the following dependencies.

```
git clone https://github.com/ros/common_msgs.git
git clone https://github.com/ros-perception/image_transport_plugins.git
git clone https://github.com/ros-perception/vision_opencv
git clone https://github.com/ros/dynamic_reconfigure.git
git clone https://github.com/ros-perception/image_common.git
git clone https://github.com/ros/diagnostics.git
git clone https://github.com/ros/bond_core.git
git clone https://github.com/ros/ros_comm.git
```

Use rosdep in ROS workspace
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build all.
```
catkin_make
```


## Usage

Launch the camera
```
roslaunch raspicam_node camerav2_410x308_30fps.launch
```

Launch the emotion detection system
```
roslaunch emotion_detection_ros emotion_detection_ros.launch
```

## Configuration parameters
### ROS configuration
You can change the names of subscribers and publishers, and other configuration in `emotion_detection_ros/config/ros.yaml`.

#### Subscribed Topics
* `camera_reading` ([sensor_msgs/CompressedImage])

Frame stream received from camera.  
You must change the name of this topic if you want to use a different camera.

#### Published Topics
* `bounding_boxes` ([emotion_detection_ros_msgs/BoundingBoxes])

Publishes a list of bounding boxes that gives information of the Class detected and position and size of the bounding box in pixel coordinates.

#### Image view
* `enable` (bool)

Controls whether you want to activate the image view or not.

* `wait_key_delay` (int)

Delay of image view.

### Model configuration
You can change some parameters of the model in `emotion_detection_ros/config/model.yaml`.

* `algorithm` (string)

Indicates the algorithm of the model to be used.  
Valid arguments: SVM, KNN and MLP.

* `max_num_faces` (int)

Indicates the maximum number of faces to be detected.

---
[1] Kanade, T., Cohn, J. F., & Tian, Y. (2000). Comprehensive database for facial expression analysis. Proceedings of the Fourth IEEE International Conference on Automatic Face and Gesture Recognition (FG'00), Grenoble, France, 46-53.

[2] Lucey, P., Cohn, J. F., Kanade, T., Saragih, J., Ambadar, Z., & Matthews, I. (2010). The Extended Cohn-Kanade Dataset (CK+): A complete expression dataset for action unit and emotion-specified expression. Proceedings of the Third International Workshop on CVPR for Human Communicative Behavior Analysis (CVPR4HB 2010), San Francisco, USA, 94-101.
