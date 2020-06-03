# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

# Objectives

"PID controllers" - some readers will have flashbacks to their first year of engineering school, with mathematical models, phase diagrams and stability calculations.
That's not what we'll be doing here.
This project will implement a simple PID controller in code, and put PID controllers to use, managing a car's throttle and steering input to maintain the car at a target speed and on a target trajectory as it drives around a race circuit.

## Control: Throttle and Steering

The implementation of a PID controller can be found in [src/PID.cpp](src/PID.cpp). A discrete PID controller receives an error value in each time step.
This is used (with prior state) to compute proportional, integral and differential terms respectively, and to return a new control variable.
With appropriately tuned P, I and D coefficients, this allows for reduction in the error and convergence on a target state.

To drive the car around its simulator track, two PID controllers were used. The first was used to control the throttle and pursue a pre-set target speed.
Here, the error term was the difference between the vehicle's actual speed and its target speed.
In this case, the D term was set to zero making this effectively a PI controller. The P coefficient was 0.1 and the I coefficient was 0.002.
These values had been used in the behavioural cloning project and achieved reasonable results in maintaining a target speed, so we'll continue to use them here.

The second PID controller was used to control the steering of the simulated vehicle, keeping the vehicle close to the centre of its lane as it drives around the track.
Here, the error term was the cross track error (CTE): the latteral displacement of the vehicle from the center of the track.
In the real world this might be inferred from localization or SLAM, but this CTE value was conveniently provided by the simulator.
Some initial P, I and D coefficients for the steering PID controller were arrived through guess work and improved a little through trial and error.
Then they were improved on more systematically, and through higher speeds, using a simple parameter tuning algorithm called twiddle.

Let's first look a little more deeply at what the P, I and D terms mean. Then we'll look more closely at Twiddle. Finally, we'll look at the result.

## P, I and D

### P

The proportional term in a PID controler consists simply of a coefficient by which the error in the current time step is multiplied.
The contribution of the P term in steering, is to set the steering input proportionally in response to the vehicle's displacement from the centre.


Baseline | P increased by 20%
:-------------------------:|:-------------------------:
[![baseline](https://img.youtube.com/vi/d8QIiBMbRew/0.jpg)](https://www.youtube.com/watch?v=d8QIiBMbRew) | [![P increased](https://img.youtube.com/vi/XZ4ML110tQE/0.jpg)](https://www.youtube.com/watch?v=XZ4ML110tQE)


### I

The integral term in a PID controller consists of the total historic error, multiplied by a coefficient.
This is perhaps a little less intuitive than the P term - the integral term becomes important where there are changes in the amount of "work" required to pursue the target, e.g. because of hills, because of curves or because of a bias.
The contribution of I in steering, is to steer a little more sharply when the vehicle has been on one side of the centre for a long time.
This is important for successfully steering around corners, but it also aggravates oscillation.

Baseline | I increased by 20%
:-------------------------:|:-------------------------:
[![baseline](https://img.youtube.com/vi/d8QIiBMbRew/0.jpg)](https://www.youtube.com/watch?v=d8QIiBMbRew) | [![I increased](https://img.youtube.com/vi/6SvPIGnH1V8/0.jpg)](https://www.youtube.com/watch?v=6SvPIGnH1V8)


### D

The differential term in a PID controller consists of the difference in error between one time step and the next (while this differs from the derivative with respect to a standard time unit such as seconds, this does not matter so long as the time difference in question is constant).
This does not contribute to reducing the error directly, but rather counteracts rapidly changing error values.
The contribution of D to steering, is to reduce the severity of the steering input as the CTE is changing most rapidly, which is typically where the vehicle is close to the centre of the track.
This helps to reduce oscillation and so brings the vehicle to a smoother convergence on the center of the track, but it leaves the vehicle more likely to spin off the track on corners.

Baseline | D increased by 20%
:-------------------------:|:-------------------------:
[![baseline](https://img.youtube.com/vi/d8QIiBMbRew/0.jpg)](https://www.youtube.com/watch?v=d8QIiBMbRew) | [![D increased](https://img.youtube.com/vi/Q3edrqUKCxM/0.jpg)](https://www.youtube.com/watch?v=Q3edrqUKCxM)



---

## Twiddle: Parameter Tuning

Twiddle is a relatively simple (and easily implemented!) local parameter tuning algorithm. The actual implementation is in [stwiddle.py](stwiddle.py).

The essential idea is that for a set of parameters, a cost score or penalty is evaluated (e.g. the sum of CTE over an entire lap of the track).
In practice, I first tried an even power of CTE, and while this provided adequate PID coefficients for getting around the track, it didn't give enough penalty to small (yet stomach churning!) oscillations.
I then settled on a cost score that summed the absolute value of CTE over all time steps, but with a radically harsher penalty where the CTE magnitude exceeded a threshold suggesting the vehicle was off-track.
Where the vehicle goes off track (high CTE magnitude), the training run aborts early with an appropriately punitive penalty, so that we can progress with tuning a little more quickly.

Given a cost score for each set of parameter values, twiddle then tries small differences (both increases and decreases) to each of these parameters.
Where a change results in an improved (reduced) score, a slightly larger difference is tried in the next iteration.
Where neither an increase nor a decrease improves the score, a slightly smaller difference is tried in the next iteration.
When the (weighted) sum of all differences has fallen below a pre-set threshold (i.e. further twiddling would make only very small changes to parameters), twiddling terminates.
The result is, we hope, a reasonably well tuned set of coefficients for our PID.

The output (logs) from some twiddle runs can be found in the [twiddle_log](twiddle_log) directory.

## Results

25 mph:
[![25mph](https://img.youtube.com/vi/d8QIiBMbRew/0.jpg)](https://www.youtube.com/watch?v=d8QIiBMbRew)

30 mph:
[![30mph](https://img.youtube.com/vi/O7qMU98LGQI/0.jpg)](https://www.youtube.com/watch?v=O7qMU98LGQI)

---

# Running this Code

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
