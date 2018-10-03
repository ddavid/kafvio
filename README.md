![kafvio](/home/david/Documents/Uni/modern-cpp/final-project/images/kafvio-first-logo.png)

Final Project for the Seminar "Advanced Software Development with Modern C++" at the LMU during the Summer semester of 2018

## System Diagram

![System Diagram](/home/david/Documents/Uni/modern-cpp/final-project/images/kafvio-system-diagram.png)

### Tracking
![](http://)

#### Kalman Filter (Linear Quadratic Estimation)
##### Prediction
##### Update

### Visual Odometry
![](/home/david/Documents/Uni/modern-cpp/final-project/images/visual-odometry.png)


**Pinhole Camera Model**
![Pinhole Camera Model](/home/david/Documents/Projects/MM/Judging/images/camera-formula.png)

**Intrinsic Camera Parameters**
(c<sub>x</sub>, c<sub>y</sub>) := principal point - where optical axis intersects the image plane
f<sub>x</sub>, f<sub>y</sub> := pixel scaling factors

**Extrinsic Camera Parameters**
<strong>R</strong> := Rotation Matrix
<strong>t</strong> := translation vector
In the form (<strong>R</strong>, <strong>t</strong>) above

#### Distance Estimation
![Height Distance Estimation](/home/david/Documents/Projects/MM/Judging/images/height-distance-estimation_comp.png)
#### Object Odometry
![Object Velocities](/home/david/Documents/Uni/modern-cpp/final-project/images/object-velocities.png)

Since our assumption is that the camera will only move on a straight line on the xy-plane, we can directly infer its velocity from the objects' translation. Otherwise we would have to take the longitudinal and rotational components of the velocity into consideration.

obj_vel := &Delta;pos &div; &Delta;t

and
obj_vel = cam_vel
![Object Velocity](http://mathurl.com/ya2ds8p5)
