# Model Predictive Controller

Controller for a self-driving car's steering and throttle commands.

<img src="mpc-demo.gif?raw=true">

---

## Introduction

This project is part of
[Udacity's Self-Driving Car Nanodegree program](https://www.udacity.com/drive).
The purpose of this project is to build a Model Predictive Controller (MPC). The
controller is tested on an Udacity simulator, which provides cross-track error
(CTE), speed and steering angle data via a websocket connection. The car drives
autonomously around the simulator track by using steering and throttle commands
from the MPC.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac:
  [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows:
  [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make -
  [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See
    [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more
    details.

* **Ipopt and CppAD:** Please refer to [this document](install_Ipopt_CppAD.md)
for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already
part of the repo.
* Simulator. You can download these from the
[releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the
data sent back from the simulator.


## Build and Run Instructions

1. Clone this repo.
2. Scripts provided in this repo could be used in the following order to build
and run the PID controller.

```bash
./build.sh  # Cleans any existing build and builds the MPC.
./run.sh    # Runs the MPC; connects to the simulator via uWebSockets.
```

## Discussion

This section discusses how the controller was set up, including some of its
parameters like time-horizon, time-step-length, etc.

### Controller Model

For setting up an MPC we'd require a dynamic-model which can predict the
trajectory of the vehicle, given its current state.

The state of the vehicle consists of the following variables.

```
1. 2D location coordinates : x and y
2. Orientation or Heading : psi
3. Speed : v
4. Cross track error : cte
5. Error in orientation : epsi
```

Control inputs (actuators) include:

```
1. Steering-angle : delta
2. Throttle and brake : a
```

Given the above state and actuators formulation, we use a simple kinematics model
to predict the next state(s) of the vehicle. Please note that we aren't using
a complex dynamics model which would consider factors like the forces (lateral,
longitudinal) acting between the tyres and the road. Kinematics model equations
that we use are given below.

* x<sub>t+1</sub> = x<sub>t</sub> + (v<sub>t</sub> \* cos(psi<sub>t</sub>) \* dt)
* y<sub>t+1</sub> = y<sub>t</sub> + (v<sub>t</sub> \* sin(psi<sub>t</sub>) \* dt)
* psi<sub>t+1</sub> = psi<sub>t</sub> +
(v<sub>t</sub> \* delta<sub>t</sub> \*  dt / L<sub>f</sub>)
* v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt
* cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> +
(v<sub>t</sub> \* sin(epsi<sub>t</sub>) \* dt)
* epsi<sub>t+1</sub> = epsi<sub>t</sub> - (psiDesired<sub>t</sub>) +
(v<sub>t</sub> \* delta<sub>t</sub> \*  dt / L<sub>f</sub>)

In the above equations,
* `f(x<sub>t</sub>)` is the function, in this case a 3rd degree polynomial, which
fits over the given waypoints. Typically these waypoints are supplied by a
path-planning algorithm; in this project they're provided by the simulator.
* `dt` denotes the time-interval between two consecutive steps in the
time-horizon as discussed in section [here](tuning-the-controller-parameters).
* `L<sub>f</sub>` is the shortest distance from the center-of-gravity of the
vehicle to its front axle.
* `psiDesired` is the desired steering-angle based on the input waypoints given
to the controller.

As with any other model, MPC has an objective with a set of constraints.
Objective for MPC here is to find the optimal steering and throttle command,
with the below constraints.

1. State based constraints : Cross track error (CTE), orientation error
(epsi), and a penalty if the vehicle slows down below a reference velocity (set
to 100). Please refer to statement lines 39 to 43 in
[src/MPC.cpp](./src/MPC.cpp), which shows their squared-sum error values
accumulated in the constraints.

2. Actuator constraints : Shown in lines 46 to 49 in [src/MPC.app](./src/MPC.cpp)
is the costs associated with actuators i.e. steering-angle `delta` and throttle
`a`.

3. Abrupt actuator constraints : For producing a smooth transition between
actuations, we penalize consecutive actuations as given in statements in
[src/MPC.cpp](./src/MPC.cpp) lines 52 to 55.


### Tuning the controller parameters

For the MPC to work, we've to tune few parameters applicable to the
**time-horizon** and how fast we want the model to predict/control the
trajectory. Two such parameters are the `time-step length` (N) and
`time-step interval` (dt). In this project we set them to 10 and 0.1
(in seconds) respectively after bit of trial and error compensating for how fast
the commands (steering-angle and throttle) are propagated to the simulated
vehicle and how quickly those actions take place.

### MPC preprocessing and Polynomial fitting

When the waypoints are supplied to the MPC, they're transformed to suit the
vehicle's coordinates. In this project, only the orientation/heading angle
has to reversed and the 2D coordinate points transformed accordingly. Statements
in [src/main.cpp](./src/main.cpp) lines 100 to 106 apply this transformation.

A third-degree polynomial is fitted to the input waypoints, as shown in line
[here](./src/main.cpp#L109). This polynomial's coefficients are used for:

* Finding the `CTE` and `epsi`.
* Solving for finding the predicted trajectory.

### Model Predictive Control with Latency

Often it happens that a certain amount of latency exists between the time when
actuator commands are supplied to the vehicle and the time when they're actually
executed. The latency associated with the simulator in this project is 100
milliseconds. This latency is compensated for in the controller when we
initialize the state with the delay interval, based on kinematics model
discussed above, as shown [src/main.cpp](./src/main.cpp) lines 123 to 128.
