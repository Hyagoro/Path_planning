# Path planning

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

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every
 .02 seconds. The units for the (x,y) points are in meters and the spacing
  of the points determines the speed of the car. The vector going from a
   point to the next point in the list dictates the angle of the car.
    Acceleration both in the tangential and normal directions is measured along
     with the jerk, the rate of change of total Acceleration.
      The (x,y) point paths that the planner recieves should not have a total acceleration
       that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.
        (NOTE: As this is BETA, these requirements might change. Also currently
         jerk is over a .02 second interval, it would probably be better to average 
         total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path
 planner returning a path, with optimized code usually its not very long
  maybe just 1-3 time steps. During this delay the simulator will continue
   using points that it was last given, because of this its a good idea
    to store the last points you have used so you can have a smooth transition.
     previous_path_x, and previous_path_y can be helpful for this transition since
      they show the last points given to the simulator controller with the processed
       points already removed. You would either return a path that extends this previous
        path or make sure to create a new path that has a smooth transition with this last path.

#### Model

The first part oh the pipeline is to calculate trajectory points with `getXY` function 30m ahead of the car.
Next, we pass those coordinates into car space and then fit a `spline` function to get a smoother path.

```cpp
// create spline
tk::spline s;

// set(x,y) to the spline
 s.set_points(x_points, y_points);
```

As noticed before, we work with sensor fusion. For each vehicle in the map, I determine if there is a car in 
in front, at the left or at the right of our vehicle by exploiting `vx`, `vy` and `car_s`. 

```cpp
double vx = car_details[3];
double vy = car_details[4];
double check_speed = sqrt(vx * vx + vy * vy);
double check_car_s = car_details[5];
```

I use 30m for vehicle in our lane (for satefy), 20m (front) and (10m) back for vehicle at he right and left. I used smaller 
values for left and right to avoid being stuck for too long between a border and the middle lane.

A state machine is used in order to decide if the car need to change lane or not.
The priority is to stay on the actual lane while right lane or left lane contains vehicles.

States:
- "KL" - Keep Lane
- "LCL" / "LCR"- Lane Change Left / Lane Change Right
- "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right

I also used a cost function to give a priority to each potential new state to make the best choice. This cost
function takes in input the state, the content of sensor fusion. 

```cpp
double Vehicle::calculate_cost(string state, vector<vector<double>> sensor_fusion, int prev_size);
```

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

 