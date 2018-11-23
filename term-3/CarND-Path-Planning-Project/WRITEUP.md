# Path Planning

## Compilation
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Valid Trajectories

### The car is able to drive at least 4.32 miles without incident..
A video where the car drives for 10 minutes (8 miles) can be found [here](https://drive.google.com/open?id=1mopv48IfcDC2EToGr2wewJQ2nC0WXyf6).

### The car drives according to the speed limit.
The car tries to stay as close to the speed limit as possible without exceeding it. Average speed after 10 minutes was 48 mph.

### Max Acceleration and Jerk are not Exceeded.
The car never exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

### Car does not have collisions.
The car does not come into contact with any of the other cars on the road.

### The car stays in its lane, except for the time between changing lanes.
The car never leafes the track.

### The car is able to change lanes
The car can smoothly change lanes when it makes sense to do so.

## Implementation

The code is based on the [Path Planning Walkthrough video](https://www.youtube.com/watch?v=7sI3VHFPP0w). 

A simplified prediction algorithm is used where three variables (`car_left`, `car_ahead`, `car_right`) represent whether the ego vehicle is surrounded by other cars. For each element in `sensor_fusion` the next position for this vehicle is calculated `predicted_s` (line 268), and if this vehicle is too near the ego vehicle, the flags are set (line 271-277).

In the next step, the behavioral planning, there are only three possible states (keep in lane, left lane change and right lane change). The ego vehicle will keep in the lane as long as `car_ahead == false`. When `car_ahead == true` it will try to change lane.

In the trajectory planning step a smooth path is planed using the spline library as suggested in the [Path Planning Walkthrough](https://www.youtube.com/watch?v=7sI3VHFPP0w).


## Reflection

After watching the video of the [Path Planning Walkthrough video](https://www.youtube.com/watch?v=7sI3VHFPP0w) I liked the proposed idea and iterated on it. The current implementation of the "cost" function could be expanded on to not only include the cars right next to the ego vehicle but also the cars a few meters down the road. With this the car should only change into a lane that improves its forward progress, right not it can get slowed down by changing into a lane with a lot of traffic.
