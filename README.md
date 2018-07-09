# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


# Approach
The current project solves the given problem of driving car with conditions given in the problem statement. Following details the approach.
## Point generation
As the simulator expects (x, y) co-ordinates for the car's next location, the program essentially generates these co-ordinates. The simulator is supposed to get these co-ordinates (where the car should be) in every 0.02 seconds.Hence, a large list of co-ordinates results in longer path to travel in 0.02 seconds. In other words, path length determines car's velocity and vice versa. Given this, I have essentially approached the problem in generating these (x, y) co-oridnates or so-called way points for every 0.02 seconds. For simplicity, I have used frenet co-ordinates to generate the path. Specifically, for each 0.02 seconds, the program generates two way points each 30 meters away from each other. Spline tool is then used to compute a smooth path between these points.

Car's velocity determines the number of way points to be generated. Also, the lane (one of co-ordinate in frenet co-ordinate) determines where the car is going to be be. Hence, the task is now to come up with the lane that the car should be driven and the velocity. I have used Finite State Machine based approach for generating lane and velocity.

## FSM based velocity and lane
The idea is to observe the behivor of other cars and adjust our car's velocity and lane. At each point of time, the car can either decide to drive in its own lane or change lanes. As the lane changing requires velocity adjustment to the target lane, the car can be in a state where it's deciding whether it should change lanes or not. Given that there are two possiblities for lane change (left and right), the car can be in one of the following states, 1. Keep driving in Lane. 2. Prepare Lane Change Left, 3. Prepare Lane Change Right, 4. Lane Change Left, 5. Lane Change Right. State list and transitioning is detailed in Vehicle.cpp and Vehicle.h.

These states conflict with each other as at times, staying in the lane is better than going in the left lane. However, driving in the current lane may result in un-necessary delays. State transition is decided based on a cost function. Cost function essentially determines whether it is better to stay in the current lane or change lanes. First, I describe possible states and then the cost function.

### Keep Lane
In this state, the program tries to see whether it's good for the car to stay in the current lane. One important aspect here is adjusting the speed of the car. There may be a car driving in the current lane at lower speed, the car needs to adjust its speed based on it. This is decided with two variables. 1. Buffer and 2. Speed of the car in front. As the car approaches to the car in front, buffer reduces, resulting in decreased car velocity. Also, the car need not decrese the velocity once it finds itself with the velocity of the car in front. With lower buffer, the car tries to reudce more velocity.

### Prepare Lane Change
In this state, the car tries to adjust its velocity to the left/right lane's car. The car keeps itself in this state until it finds a clear left/right lane to move into. The velocity is adjusted according to either the vehicle in front or the vehicle in front in the left/right lane. In other words, in this state, car has already decided that lane change is the best strategy. For lane change, it may need to reduce its velocity to match to the velocity of the car in left/right lane, at the same time, not hitting the car in back or front in current lane.

### Lane Change
As soon as car finds an empty spot in the target lane (left or right), it gets into that lane. One important difference in this state from other two states is that it does not increase the velocity and reduce its velocity by at least a constance (since it may be turning).

In both Prepare Lange Change and Lane Change state, the car checkes far left and far right lane if available and decides whether it should be moving in those lanes as well.

## Cost function
These decisions are based on a cost function which essentially checks for the following costs,
1. Possible collision
2. Front Buffer
3. Velocity change required
4. Velocity that can be achieved
5. Back buffer

These costs are numbered according to their importance in deciding the state. I.e., cost associated with 1 is considered higher than cost associated with 2 and so on.

The first and foremost is to check whether moving to the left/right lane can result in a collision. If it does, then it is better to stay in the current lane. This also has an implication in sudden events like, a car decides to abruptly change its lane to current lane. In this case, there may be a possible collision if we stay in the current lane, hence it is better to move to the left or right lane even if it results in the decreased velocity. This cost is checked based on both, front buffer and back buffer.

Another important cost is front buffer, it is good to drive in the lane that has a lot of empty space. If there is a car in a lane with a lot of empty space, going at relatively slower speed, it is still good to drive in that lane than being a lane with less buffer but relatively higher speed. As with larger buffer space, we hae a better probability to find another lane and move into that lane. Hence, this is cost is considered more important than other costs like target velocity of the car and velocity change required.

Velocity change required is also an important measure since we may not want to drive in a lane requiring steep velocity changes. Hence, this cost is considered. Similarly, we would want to drive in a lane where we can get a good velocity i.e. drive in the lane where the fastest car or no car is driven. This is decided by velocity that can be achieved cost.

Last, we want to drive in the lane where we have enough buffer between our car and the car behind it. Hence, the consideration of this cost. Cost.h and Cost.cpp has code that emulates the behivior as described.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
