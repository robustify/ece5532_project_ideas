# IGVC Simulation

This project uses the `igvc_flatland` repository available on Oakland Robotics' Bitbucket account to simulate the IGVC competition. There is documentation about the simulator there: [https://bitbucket.org/oaklandrobotics/igvc_flatland](https://bitbucket.org/oaklandrobotics/igvc_flatland). Follow the instructions to clone and compile everything you need to run the simulation. You might get errors trying to compile it because of some missing dependencies. Run `deps.bash` from the root of your ROS workspace folder to automatically install these dependencies before trying to compile, or after if you do get errors:

```
cd ros
deps.bash
```

In addition to setup instructions, the documentation explains what happens when running the simulation, which topics are advertised and subscribed to, and which launch files exist to start the simulation.

## Objectives

### Level 1: Complete the Basic Course

- Configure the ROS navigation stack to work with the simulated IGVC robot.
- Configure the costmaps to take inputs from the simulated LIDAR scan and the camera "LIDAR" scan to avoid both lines and barrels at the same time.
- Complete both North and South runs on the basic course.

### Level 2: Complete the Advanced Course

- Apply the same system to navigate the advanced course which has more obstacles, and also requires reaching 6 additional waypoints in the middle of the course.
- Complete both North and South runs on the advanced course.
