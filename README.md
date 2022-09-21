# CompRobo Warmup Project 2022

Explain what this project was, list contributors

# Teleop

![](pictures/teleop_demo.gif)

## Approach


## Structure
structure - like ros topics, publishers, subscribers

## Limitation
what can be improved

# Drive Square

![](pictures/drive_square_demo.gif)

## Approach
how this is created, basic concept

## Structure
structure - like ros topics, publishers, subscribers

## Limitation
what can be improved

# Wall Follower

![](pictures/wall_follower_demo.gif)

## Approach
how this is created, basic concept

## Structure
structure - like ros topics, publishers, subscribers

## Limitation
what can be improved

# Person Follower

![](pictures/person_follower_demo.gif)

## Approach
how this is created, basic concept

## Structure
structure - like ros topics, publishers, subscribers

## Limitation
Since the robot is locating person by getting the centroid of the LIDAR measurement, there are multiple issues and limiations.

- It cannot distinguish between a person (moving object) from other objects (stationary). One approach to solve this problem is to wait for multiple LIDAR scans in a duration and only follow the object that have moved more than a certain distance. 
- It cannot detect multiple objects at a same time, meaning that the target position might not be accurate once there are many objects. This can be avoided by getting centroid for different objects (for example define as different object if the LIDAR scan returned 0 for certain index) and looking for the closest object centroid as destination.
- The robot will stay forever in one location until it finds an object to follow. To prevent this, simple behavior like rotating or moving around can be implemented to actively find a new target.

# Obstacle Avoidance

![](pictures/obstacle_avoidance_demo.gif)

## Approach
how this is created, basic concept

## Structure
structure - like ros topics, publishers, subscribers

## Limitation
what can be improved

# Finite State Control

![](pictures/finite_state_controller_demo.gif)

## Approach
how this is created, basic concept

## Structure
structure - like ros topics, publishers, subscribers

## Limitation
what can be improved