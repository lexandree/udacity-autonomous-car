# Extended Kalman Filter Project

This project consists of implementing an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) with C++. A Term 2 simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of the position and velocity of an object, and the Extended Kalman Filter (EKF) must fusion those measurements to predict the position of the object. The communication between the simulator and the EKF is done using the [WebSocket](https://en.wikipedia.org/wiki/WebSocket) [implementation](https://github.com/uNetworking/uWebSockets).
The starter code is provided by Udacity and all instructions could be found here https://github.com/udacity/CarND-Extended-Kalman-Filter-Project.

I have used Udacity workspace and here the results for both test tracks:

Track 1 RMSE

![](https://github.com/lexandree/udacity-autonomous-car/blob/master/project5/RMSE1.PNG)

Track 2 RMSE

![](https://github.com/lexandree/udacity-autonomous-car/blob/master/project5/RMSE2.PNG)

The important difference of my implementation is a clear differentiating of the operating class and the EKF itself using incapsulation paradigm of the c++. This makes the FusionEKF class easy to observate and to change. The KalmanFilter class encapsulated all the data and matrices it needs and provides all the necessary interface to get the result, excluding direct access to private variables.
