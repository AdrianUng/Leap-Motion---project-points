# Project Leap Motion (3D points) onto a Webcam's 2D plane

![](https://github.com/AdrianUng/Leap-Motion-project-points-onto-image/blob/master/other%20imgs/gif1.gif)

## Packages used/required:
* OpenCV version 3.4.2
* NumPy version 1.15.4
* [Leap Motion Python API for Python](https://developer-archive.leapmotion.com/documentation/index.html)
* have Leap Motion SDK installed on machine
* a webcam (in my case a Logitech Pro 9000)

## General description of the project:
Using 3D coordinates relative to Leap Motion, project the points onto a webcam's 2D image plane.

By running the script in *project_points.py* the following are achieved:
* reading **frames** from the webcam (Read [documentation](https://developer-archive.leapmotion.com/documentation/python/index.html) for more info)
* reading **coordinates** relative to the Leap Motion (Read [documentation](https://developer-archive.leapmotion.com/documentation/python/index.html) for more info)
* projecting the 3D coordinates onto the webcam's image plane [1]:
  * requires previously determined information -> the webcam's **Intrinsic Matrix** (easily obtainable by calibrating the camera)
  * requires **Extrinsic Matrix**, containing the **Rotation Matrix (R)**, as well as the **Translation Vector (t)**
  * in this case the **R** is considered the identity matrix of size 3. The **t** is also made up of zeros
![](https://github.com/AdrianUng/Leap-Motion-project-points-onto-image/blob/master/projection_formula.PNG)
* The setup is made up of a Leap Motion placed on top of the webcam:

![](https://github.com/AdrianUng/Leap-Motion-project-points-onto-image/blob/master/other%20imgs/setup.png)

* The coordinate systems of the 2 devices are different:

![](https://github.com/AdrianUng/Leap-Motion-project-points-onto-image/blob/master/other%20imgs/axis_coordinates.png)

* The camera feed will be displayed *ONLY* if a hand is detected in front of the Leap Motion
* This script was developed to save info to a destination, in this case considered *session0/id0_inst0/'*, where:
  * **session** corresponds to an acquisition session
  * **id#** corresponds to a participant's id
  * **inst** corresponds to an instance number (e.g. open fingers, closed fingers, horizontal hand, vertical hand, etc.)
  * These numbers can be easily changed within the script, towards the end of the file
  
* To **SAVE** images and coordinates at the destination (*session0/id0_inst0/'*), press '**s**' on your keyboard
  * Image showing the overlap of projected points
  * (Clean) image acquired with webcam
  * 3D coordinates as acquired by the Leap Motion, in XML format
  * 2D coordinates, in XML format, as projected using the matrices defined earlier
  
* To **EXIT** from the script, press '**q**' on your keyboard

**REFERENCES**

[1] Z. Zhang, “A Flexible New Technique for Camera Calibration (Technical Report),” IEEE Trans. Pattern Anal. Mach. Intell., vol. 22, no. 11, pp. 1330–1334, 2002.

  

  


