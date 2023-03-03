# Audibot Adaptive Cruise Control (ACC)

This project uses two instances of the Audibot simulator that was used throughout the course, and the goal is to implement adaptive cruise control where one car follows the other and regulates its following distance.

## Objectives

Here are some sample objectives for the ACC project split into different levels of complexity.

### Level 1: Implement core system with ideal measurements

- Look up the states of the two cars' Gazebo models, extract their positions, and compute the exact distance between them.
- Use the computed distance as input to an algorithm that controls the speed of the following car such that it maintains a relative distance to the lead vehicle.
- The `audibot_path_following` node publishes to a `geometry_msgs/Twist` topic to hold a constant speed and to follow the lane markings. To impose a different speed for ACC, remap this topic to the ACC node and then replace the `linear.x` field with a new speed, keeping the `angular.z` unchanged.

### Level 2: Use a LIDAR sensor to detect the lead vehicle

- Process the LIDAR scan coming from the simulated sensor on the following vehicle (`/ego_vehicle/laser/scan`) to detect the lead vehicle and compute its distance.
- Use this computed LIDAR distance to maintain a following distance instead of using the model state lookup method from Level 1.

### Level 3: Use the camera to detect the lead vehicle

- Process the image from the following vehicle's camera to detect the lead vehicle.
- Use trigonometry and the known height and orientation of the camera to estimate the distance to the lead vehicle.
- Pass the computed camera distance to the ACC controller from Level 1.

## Getting Started
To start the Audibot ACC simulation, you can use the provided `run_simulation.launch` file:

```
roslaunch audibot_acc run_simulation.launch
```

Then, open `rqt_graph` and analyze how the provided system is set up.

## Modifying the Simulation to Include ACC

There are two instances of the path following and twist control nodes in different namespaces: one for the leading vehicle (`target_vehicle`) and one for the following vehicle (`ego_vehicle`). Each `audibot_path_following` node publishes commands directly to the `audibot_twist_control` nodes to follow the lane center and hold a constant speed.

To implement ACC, run your node in the `ego_vehicle` namespace and remap the output topic from the path following node (`/ego_vehicle/cmd_vel`) to the input topic of your ACC node. Then remap the output topic from your ACC node to `/ego_vehicle/cmd_vel` so it can control the vehicle instead.

## Looking up the Exact Positions of the Vehicles

While Gazebo is running, the state of every model in the simulation is continuously published on the `/gazebo/model_states` topic, which contains `gazebo_msgs/ModelStates` messages.

The `gazebo_msgs/ModelStates` message contains three array fields: `name`, `pose`, and `twist`. The lengths of each array matches the number of models in the simulation. To find the position of one of the vehicles, find the array index in `name` that has the name of the model, then use that index to extract the `pose` element corresponding to it.

Do the same for the other vehicle, and then you can calculate the distance between the vehicles for inputting into the ACC algorithm.
