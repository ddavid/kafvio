![kafvio](https://imgur.com/2kWdeMj.png)

Final Project for the Seminar "Advanced Software Development with Modern C++" at the LMU during the Summer semester of 2018

## System Diagram

![System Diagram](https://imgur.com/hLfMSFH.png)

### Tracking

![](http://)

#### Kalman Filter (Linear Quadratic Estimation)

![Kalman Filter Diagram](https://imgur.com/FogSISI.png)

##### Prediction
1. Use process model to predict state at the next time step
2. Adjust belief to account for the uncertainty in prediction

![Prediction Equations](http://mathurl.com/y8reaut5.png)
##### Update
1. Compute residual between prediction and measurement
2. Compute scaling factor based on accuracy of both prediction and measurement
3. Set state according to scaling factor
4. Update belief in the state based on measurement certainty

![Update Equations](http://mathurl.com/y7cehhk4.png)

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
