# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Introduction
The purpose of this project is to implement a simple PID controller with Twiddle Parameter optimization. 

## Reflection
A proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism (controller) widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (sometimes denoted P, I, and D respectively) which give their name to the controller type. In this project we use the cross-track error (cte) as the error and apply PID controller to calculate the Steering angle. 

### P (proportional)
In the P term of the algorythm, The steering angle will be calculated in propotion to the error. This leads to a lot of Overshoot.

### I (Integral)
In the integral term of PID Controller, the steering angle increases in relation not only to the error but also the time for which it has persisted. This this case, the we calculate the sum of all the errors and calculate the new angle with regard to the sum. I term alone can bring the error down to 0 eventually but it it will start slow and react quickly when the time passes. 

### D (Derivative)
The D term of the controller does not consider the error, but the rate of change of the error. This term tries to bring the rate of change of the error to zero which means to keep the error on a straight line. 

Taking all three terms together, the controller tries to bring the error down to 0 and keep it that way hense the car in our case will be in the center of the lane all the time. 

(Unfortunately I was not able to capture Video segments to demonstrate as my Simulator for some reason didnt let me record.)

## Hyperparameters
I used Twiddle to choose the hyper parameters in my project. I implemented Twiddle in PID.cpp file under the Update function.  My final parameters are as follows

Kp: 0.0867412 
Ki: 0.000973738 
Kd: 4.00377

## Running code with Twiddle
I have implemented twiddle in PID::Update(double cte) function. To execute the program with Twiddle, pleasse make the following changes to the code and recompile.

1. Uncomment line 38 in main.cpp
2. Comment line 42 in main.cpp
3. Set PID::doTwiddle flag to false in line 29 in PID.cpp

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
