# Project Writeup

This document addresses the grading rubric, discussing the following:

* Model description including its state, actuators and update equations
* Timestep Length and Elapsed Duration settings
* MPC Preprocessing and Polynomial Fitting
* Model Predictive Control with Latency

## Controller Model

For setting up an MPC we require a model which can predict the
trajectory of the vehicle, given its current state.

The state of the vehicle, under this model, consists of the following variables.

```
1. Location coordinate x
2. Location coordinate y
3. Orientation or Heading : psi
4. Speed : v
5. Cross track error : cte
6. Error in orientation : epsi
```

Control inputs (actuators) include:

```
1. Steering-angle : delta  # is capped between the range -25 to 25 degrees.
2. Throttle and brake : a  # is capped between the range -1 (braking) to 1
(accelerating).
```

Given the above state and actuators' framework, we use a simple
**kinematics model** to predict the next state(s) of the vehicle. Please note
that we aren't using a complex dynamics model which would consider factors like
the forces (lateral, longitudinal) acting between the tyres and the road. This
dynamics model is out of scope for this project. Kinematics model equations that
we use to update the state variables are given below.

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
* **(t+1)** is the next time-step, **t** is the current time-step.
* **f(x<sub>t</sub>)** is the function, in this case a 3rd degree polynomial,
which fits over the given waypoints. Typically these waypoints are supplied by a
path-planning algorithm; in this project they're provided by the simulator.
* **dt** denotes the time-interval between two consecutive steps in the
time-horizon as discussed in section [here](tuning-the-controller-parameters).
* **L<sub>f</sub>** is the shortest distance from the center-of-gravity of the
vehicle to its front axle.
* **psiDesired** is the desired steering-angle based on the input waypoints
given to the controller.

As with any other model, MPC has an objective with a set of constraints.
Objective for MPC here is to find the optimal steering (`delta`)and throttle
commands (`a`), with the below constraints.

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


### Timestep Length and Elapsed Duration settings

For the MPC to work, we've to tune few parameters related to the
how far ahead in future we want to predict the vehicle's trajectory (Timestep
length: N) and how fast we want the model to predict/control the trajectory
(Elapsed duration: dt). Together these two parameters determine the
**time-horizon**.

A higher `N` means that the MPC will take more computational
power since the number of variables to find/optimize is doubled (`delta` and
`a`) per step. At the same time, we have to keep the time-interval between
consecutive steps minimal in order to produce quick actuations that are suitable
to the environment (e.g. track's corners/turns).

In this project we pick the horizon duration to be for 1 second i.e. the
optimizer will keep adjusting the predicted trajectory every other second, which
is reasonable given the speed we're aiming for the vehicle's travel: ~55 mph.
For 1 second horizon, we choose number of timesteps N=10 and and time-interval
dt=0.1 seconds.

### MPC preprocessing and Polynomial fitting

When the waypoints are supplied to the MPC, they're transformed to suit the
vehicle's coordinates. In this project's simulator a positive orientation
implies a right-turn, which is opposite to the Kinematics update-equations
discussed above where positive orientation means a counter-clockwise rotation
with respect to the vehicle's current heading. To cater for this, we simply
negate the orientation value `delta` and transform the x, y coordinates
accordingly. The transformation equations' pseudocode is given below.

```
neg_psi = 0 - psi  # negate the orientation angle
for waypoint in waypoints:  # Transform all the waypoints
    x := waypoint.x  # extract x-coordinate of the waypoint
    y := waypoint.y  # extract y-coordinate of the waypoint
    dx := starting_x - waypoints.x  # x-coordinate of the vector from starting point to the waypoint
    dy := starting_y - waypoints.x  # y-coordinate of the vector from starting point to the waypoint
    trans_x := dx * cos(neg_psi) - dy * sin(neg_psi)
    trans_y := dx * sin(neg_psi) + dy * cos(neg_psi)
    Replace current waypoint coordinates with trans_x and trans_y.
end loop
```

Statements in [src/main.cpp](./src/main.cpp) lines 100 to 106 apply this
transformation.

A third-degree polynomial is fitted to the input waypoints, as shown in line
[here](./src/main.cpp#L109). This polynomial's coefficients are used for:

* Finding the `CTE` and `epsi`.
* Solving for finding the predicted trajectory.

### Model Predictive Control with Latency

Often it happens that a certain amount of latency exists between the time when
actuator commands are supplied to the vehicle and the time when they're actually
executed. The latency associated with the simulator in this project is 100
milliseconds. This latency is compensated for in the controller when we
initialize the starting state with the delay interval, based on the Kinematics
model discussed above i.e. in place of `dt` we include the delay value of 100
milliseconds. This is implemented in [src/main.cpp](./src/main.cpp),
lines 123 to 128.
