# Project 8 - PID Controller

## Goal
The goal of this project is to drive a car smoothly around a track using the distance to the center of the lane as input and control the speed additionaly.

## Environment
Prerequisites and instructions how to install project components are on the [project page](https://github.com/udacity/CarND-Controls-PID) and the websocket is [here](https://github.com/uNetworking/uWebSockets)

I have installed the environment under Udacity Ubuntu Virtual Box with default pretty old tools:
- gcc 5.4.0
- make 4.1
- cmake 3.5.1
- [Udacity Term 2 Simulator v1.45](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45)

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## [PID Controller](https://en.wikipedia.org/wiki/PID_controller)
"A proportional–integral–derivative controller (PID controller or three-term controller) is a control loop mechanism employing feedback that is widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t)as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively), hence the name." [[1](https://en.wikipedia.org/wiki/PID_controller)]

![txt][image1]

The [basic implementation](https://github.com/lexandree/udacity-autonomous-car/blob/master/project8/src/PID.cpp) derived from python lesson is pretty easy:

```cpp
void PID::Init(double Kp_, double Ki_, double Kd_, double p_error_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = p_error_;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}
```
## Parameter Tuning

Trying to tweak the parameters on my virtual machine showed too big delays. The values close to the optimal allowed the car to pass the track a few times, but the oszillations were not comfortable.  
Probably the twiddle algorithm can find the optimal settings, but in presence of uneven time delays and different response times, it will be necessary to introduce the time parameter delta t.  
I used manual setting first:  
![txt][image2]

Then I used [Ziegler-Nichols method](https://en.wikipedia.org/wiki/Ziegler-Nichols_method):  
![txt][image3]

and got parameters 0.17, 0.0003, 1.8 for terms P, I, D respectively.

## Conclusion
"The fundamental difficulty with PID control is that it is a feedback control system, with constant parameters, and no direct knowledge of the process, and thus overall performance is reactive and a compromise. While PID control is the best controller in an observer without a model of the process, better performance can be obtained by overtly modeling the actor of the process without resorting to an observer.   
PID controllers, when used alone, can give poor performance when the PID loop gains must be reduced so that the control system does not overshoot, oscillate or hunt about the control setpoint value.  
They also have difficulties in the presence of non-linearities, may trade-off regulation versus response time, do not react to changing process behavior (say, the process changes after it has warmed up), and have lag in responding to large disturbances." [[1](https://en.wikipedia.org/wiki/PID_controller)]


[//]: # (Image References)

[image1]: img/PID.jpg "PID"
[image2]: img/PID_settings.PNG "manual tuning"
[image3]: img/PID_tune.PNG "Ziegler-Nichols method"
