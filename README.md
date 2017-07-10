# CarND-MPC-Project-mwolfram

### Starting the controller

The controller can load a file called ```config.cfg``` which contains all parameters for the optimizer. The file is located in the working directory (in this case the project directory). This feature can be activated on lines 125 and 139-152 in main.cpp.

However, for the submission, it is not active and thus the config-file is not necessary.

### Preprocessing of Waypoints

Before the waypoints are processed, they are converted to car coordinates. This happens by calling the ```transformPoint``` function on each waypoint. The transformation is useful, because this way, at each point of the track I can assume that I am presented a similar set of waypoints.

### How the controller works

The controller gets the current state of the vehicle, which are the following values:

* **x**, x position of the car
* **y**, y position of the car
* **psi**, yaw orientation of the car
* **v**, velocity of the car
* **cte**, cross-track error, the distance between the center of the road and the car
* **epsi**, the orientation error between the center "line", which is the tangent of a polynomial, and the car orientation

The first three values are zeroed out, as I already transformed my waypoints into car coordinates. So I can assume that the car is at position 0,0 with orientation 0.

The velocity is given, but **cte** and **epsi** have to be calculated manually. First a polynomial has to be fit to the waypoints that we receive and that were already converted to car coordinates. Then, for **cte**, the polynomial has to be evaluated at the height of the car and compared with the car position. For **epsi** on the other hand, the derivative of the polynomial has to be evaluated and compared with the car orientation.

I pass these values to a solver that will return the actuator signals, which are steering and throttle. For this, I have to pass the solver a set of constraints that it can work wit, namely the update equations of the vehicle (starting on line 136 in MPC.cpp) and a cost function with several parameters that need to be tuned (starting on line 82 in MPC.cpp).

### Tuning controller parameters

Having the parameters in a separate file made it a lot easier to tune parameters. The file is read before each call to the solver. So far I could not detect any performance issues with this solution.

#### Chosen parameters

* **N** and **dt** were chosen to get a good lookahead on the track while not sacrificing too much calculation time. A value of 10 for **N** seemed like a good tradeoff. Lower values did not give me the lookahead I needed, with higher values the solver seemed to struggle with the time limit it was given. **dt** was set to 0.2, which gives us a lookahead if **N**x**dt** = 2 seconds. There were no obvious drawbacks in the granularity of the calculation that I chose. In fact, there was even an advantage: by setting **dt** greater than the lag time (100ms), I automatically coped with the system lag.

The chosen parameters are:

```
ref_cte: 0.0
ref_epsi: 0.0
ref_vel_: 100.0
cte_w_: 1700.0
epsi_w_: 1200.0
vel_w_: 1.0
delta_w_: 1.0
acc_w_: 12.0
delta_change_w_: 5.0
acc_change_w_: 1.0
```

My approach was the following: I started with low speeds and tried to keep the car constrained to the center of the lane, without losing too much speed in the curves. At higher speeds, I had to increase the cost of CTE and PSI errors and at the same time give the car the permission to brake a little harder (lowering acc_w).

### Latency

The latency is dealt with by setting dt to a value that is larger than the lag time. In my case dt is set to 200ms, whereas the lag time is 100ms.
