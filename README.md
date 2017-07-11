# CarND-MPC-Project-mwolfram

### Starting the controller

The controller can load a file called ```config.cfg``` which contains all parameters for the optimizer. The file is located in the working directory (in this case the project directory). This feature can be activated on lines 136 and 151-165 in main.cpp.

However, for the submission, it is not active and thus the config-file is not necessary.

### Preprocessing of Waypoints

Before the waypoints are processed, they are converted to car coordinates. This happens by calling the ```transformPoint``` function on each waypoint. The transformation is useful, because this way, at each point of the track I can assume that I am presented a similar set of waypoints.

### Dealing with Latency

To deal with the latency of the system, the state of the car is simulated forward in time, by 100ms, before passing it to the solver. This happens on lines 169-174 in main.cpp. Also, I have to memorize the actuator signals from the previous iteration.

### How the controller works

The controller gets the current state of the vehicle, which are the following values:

* **x**, x position of the car
* **y**, y position of the car
* **psi**, yaw orientation of the car
* **v**, velocity of the car in MPH
* **cte**, cross-track error, the distance between the center of the road and the car
* **epsi**, the orientation error between the center "line", which is the tangent of a polynomial, and the car orientation

The first three values are zeroed out, as I already transformed my waypoints into car coordinates. So I can assume that the car is at position 0,0 with orientation 0.

The velocity is given in MPH, so it is converted to m/sec. **cte** and **epsi** have to be calculated manually. First a polynomial has to be fit to the waypoints that we receive and that were already converted to car coordinates. Then, for **cte**, the polynomial has to be evaluated at the height of the car and compared with the car position. For **epsi** on the other hand, the derivative of the polynomial has to be evaluated and compared with the car orientation.

I forward-simulate these values by 100ms, then pass them to a solver that will return the actuator signals, which are steering and throttle. For this, I have to pass the solver a set of constraints that it can work wit, namely the update equations of the vehicle (starting on line 136 in MPC.cpp) and a cost function with several parameters that need to be tuned (starting on line 82 in MPC.cpp).

### Tuning controller parameters

Having the parameters in a separate file made it a lot easier to tune parameters. The file is read before each call to the solver. So far I could not detect any performance issues with this solution.

#### Chosen parameters

* **N** and **dt** were chosen to get a good lookahead on the track while not sacrificing too much calculation time. A value of 15 for **N** seemed like a good tradeoff. Lower values did not give me the lookahead I needed, with higher values the solver seemed to struggle with the time limit it was given. **dt** was set to 0.1, which gives us a lookahead if **N**x**dt** = 1.5 seconds.

The chosen parameters are:

```
ref_cte: 0.0
ref_epsi: 0.0
ref_vel_: 23.0 (given in m/sec)
cte_w_: 2.0
epsi_w_: 15.0
vel_w_: 1.0
delta_w_: 2500.0
acc_w_: 1.0
delta_change_w_: 5.0
acc_change_w_: 70.0
```

My approach was the following: I started with low speeds and tried to keep the car constrained to the center of the lane, without getting too much oscillation in the curves. I put a high weight on turning commands and then gave the car just the incentive that was necessary to keep to the center (cte_w) and maintain the desired orientation (epsi_w).
