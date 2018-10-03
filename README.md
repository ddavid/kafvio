![kafvio](https://imgur.com/2kWdeMj.png)

Final Project for the Seminar "Advanced Software Development with Modern C++" at the LMU during the Summer semester of 2018

## System Diagram

![System Diagram](https://imgur.com/hLfMSFH.png)

### Tracking
![](http://)

#### Kalman Filter (Linear Quadratic Estimation)
##### Prediction
##### Update

### Visual Odometry
![Visual Odometry Subsystem Diagram](https://imgur.com/fvjIRr4.png)


**Pinhole Camera Model**
![Pinhole Camera Model](https://imgur.com/fVRhJQQ.png)

**Intrinsic Camera Parameters**

(c<sub>x</sub>, c<sub>y</sub>) := principal point - where optical axis intersects the image plane

f<sub>x</sub>, f<sub>y</sub> := pixel scaling factors

**Extrinsic Camera Parameters**

<strong>R</strong> := Rotation Matrix

<strong>t</strong> := translation vector

In the form (<strong>R</strong>, <strong>t</strong>) above

#### Distance Estimation
![Height Distance Estimation](https://imgur.com/J5FOeAh.png)
#### Object Odometry
![Object Velocities](https://imgur.com/OkTN1DR.png)

Since our assumption is that the camera will only move on a straight line on the xy-plane, we can directly infer its velocity from the objects' translation and duration of timesteps. Otherwise, we would have to take the longitudinal and rotational components of the velocity into consideration.

![Object Velocity](http://mathurl.com/ya2ds8p5.png)

and

![](http://mathurl.com/yd5dsrhk.png)
